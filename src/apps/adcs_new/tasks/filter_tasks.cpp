#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "pico/time.h"

#include "macros.h"

#include "linalg.h"
using namespace linalg;
using namespace linalg::aliases;

#include "apps/adcs_new/tasks/tasks.h"
#include "apps/adcs_new/params.h"
#include "apps/adcs_new/slate.h"

#include "gnc/mekf/filter.h"
#include "gnc/utils/transforms.h"

/*
 * Bench-only substitute for the sun sensor: feed the MEKF the accelerometer
 * instead, using local "up" in ENU as the reference. Only meaningful on a
 * bench, where gravity dominates - in orbit the spacecraft is in free fall and
 * a_body is noise.
 *
 * IMPORTANT: must be 0 for flight. The flight path fuses the sun sensor.
 */
#define GRAVITY_SENSOR 0

extern slate_t slate;

/*
 * Apply one vector measurement to the shared attitude filter.
 *
 * The propagate, magnetometer-fusion and sun-fusion tasks all run at the same
 * priority with preemption and time slicing enabled, and they all read/write
 * covariance_mat_ and error_estimate_. Without this mutex a task can be
 * preempted partway through a 6x6 matrix product and the covariance is
 * silently corrupted.
 *
 * @param fusion    Per-sensor fusion state (H, K, and its own reference)
 * @param ref_eci   Expected measurement direction in ECI. Must be a unit
 *                  vector: apply_residual() differences it against the
 *                  observation directly.
 * @param obs_body  Measured direction in the body frame. Must also be a unit
 *                  vector, for the same reason.
 */
static void apply_vector_update(SensorFusion &fusion, const float3 &ref_eci,
                                const float3 &obs_body)
{
    if (xSemaphoreTake(slate.filter_mutex,
                       pdMS_TO_TICKS(FILTER_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        LOG_ERROR("[mekf] Timed out waiting for filter mutex");
        return;
    }

    fusion.set_reference(ref_eci);
    fusion.compute_sensitivity(slate.attitude_filter.quat_);
    fusion.compute_gain(slate.attitude_filter.covariance_mat_);
    fusion.propagate_covariance_sensor(slate.attitude_filter.covariance_mat_);
    fusion.apply_residual(obs_body, slate.attitude_filter.error_estimate_);
    slate.attitude_filter.reset_error();

    xSemaphoreGive(slate.filter_mutex);
}

void vTaskMagnetometerFusion(void *) {
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_MAGNETOMETER_FUSION))
        TASK_LOOP_MS(100) // 10 Hz

        // compute_B() leaves b_eci untouched when it bails out, so a stale
        // reference would otherwise be fused as if it were current.
        if (!slate.b_field.valid)
        {
            LOG_DEBUG("[MAGNETOMETER FUSION] No valid IGRF reference");
            continue;
        }
        if (!slate.magnetometer_data.magnetometer_data_valid)
        {
            LOG_DEBUG("[MAGNETOMETER FUSION] No valid magnetometer reading");
            continue;
        }

        // b_eci is already the normalized ENU -> ECEF -> ECI reference;
        // compute_B() runs that chain internally, so don't redo it here.
        // b_body is the calibrated body-frame unit vector from the driver.
        apply_vector_update(slate.magnetometer_fusion,
                            slate.b_field.b_eci,
                            slate.magnetometer_data.b_body);

        LOG_INFO("[MAGNETOMETER FUSION] expected = [%f, %f, %f]",
                slate.magnetometer_fusion.ref_body_.x,
                slate.magnetometer_fusion.ref_body_.y,
                slate.magnetometer_fusion.ref_body_.z
                );
        LOG_INFO("[MAGNETOMETER FUSION] observed = [%f, %f, %f]",
                slate.magnetometer_data.b_body.x,
                slate.magnetometer_data.b_body.y,
                slate.magnetometer_data.b_body.z
               );
    }
}

void vTaskSunVectorFusion(void *) {
#if GRAVITY_SENSOR
    const float3 reference_enu = {0.0f, 0.0f, 1.0f};
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_SUN_VECTOR_FUSION))
        TASK_LOOP_MS(50) // 20 Hz

        if (!slate.imu_data.imu_accel_valid)
        {
            LOG_DEBUG("[GRAVITY FUSION] No valid accelerometer reading");
            continue;
        }
        if (!slate.gps_data.gps_data_valid)
        {
            LOG_DEBUG("[GRAVITY FUSION] No valid fix for the ENU frame");
            continue;
        }

        const float3 lla = {slate.gps_data.gps_lat,
            slate.gps_data.gps_lon,
            slate.gps_data.gps_alt};
        const float3 ref_ecef = enu_to_ecef(reference_enu, lla);
        const float3 ref_eci =
            normalize(ecef_to_eci(ref_ecef, slate.gps_data.MJD));

        const float3 a_obs = normalize(slate.imu_data.a_body);

        apply_vector_update(slate.sun_vector_fusion, ref_eci, a_obs);

        LOG_INFO("[GRAVITY FUSION] expected = [%f, %f, %f]",
                slate.sun_vector_fusion.ref_body_.x,
                slate.sun_vector_fusion.ref_body_.y,
                slate.sun_vector_fusion.ref_body_.z
                );
        LOG_INFO("[GRAVITY FUSION] observed = [%f, %f, %f]",
                a_obs.x,
                a_obs.y,
                a_obs.z
               );
    }
#else
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_SUN_VECTOR_FUSION))
        TASK_LOOP_MS(50) // 20 Hz

        // sun_vector_eci is derived from the UTC date/time in the fix.
        if (!slate.gps_data.gps_data_valid)
        {
            LOG_DEBUG("[SUN VECTOR FUSION] No valid fix for the ECI reference");
            continue;
        }
        // False in eclipse, or whenever fewer than three photodiodes are lit
        // well enough to solve for a direction. Fusing the zero vector it
        // returns in that case would drag the attitude estimate apart.
        if (!slate.sun_sensor.sun_vector_valid)
        {
            LOG_DEBUG("[SUN VECTOR FUSION] No valid sun vector");
            continue;
        }

        // compute_sun_vector_eci() emits an analytic unit vector, and
        // sun_sensors_to_vector() emits a normalized body-frame direction.
        apply_vector_update(slate.sun_vector_fusion,
                            slate.sun_vector.sun_vector_eci,
                            slate.sun_sensor.sun_vector_body);

        LOG_INFO("[SUN VECTOR FUSION] expected = [%f, %f, %f]",
                slate.sun_vector_fusion.ref_body_.x,
                slate.sun_vector_fusion.ref_body_.y,
                slate.sun_vector_fusion.ref_body_.z
                );
        LOG_INFO("[SUN VECTOR FUSION] observed = [%f, %f, %f]",
                slate.sun_sensor.sun_vector_body.x,
                slate.sun_sensor.sun_vector_body.y,
                slate.sun_sensor.sun_vector_body.z
                );
    }
#endif
}

void vTaskPropagate(void *) {
    stdio_flush();
    absolute_time_t t_prev = get_absolute_time();
    for(;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_PROPAGATE))
        TASK_LOOP_MS(20) // 50 Hz

        // Use the elapsed time rather than a hard-coded 0.02f: TASK_LOOP_MS is
        // a relative delay, so the real period is 20 ms plus however long the
        // body took.
        const absolute_time_t t_now = get_absolute_time();
        float dt = absolute_time_diff_us(t_prev, t_now) * 1e-6f;
        t_prev = t_now;

        // A scheduling hiccup must not inject a huge dt into the covariance.
        if (dt <= 0.0f || dt > PROPAGATE_MAX_DT_S)
        {
            LOG_DEBUG("[propagate] Discarding implausible dt = %f s", dt);
            continue;
        }

        if (xSemaphoreTake(slate.filter_mutex,
                           pdMS_TO_TICKS(FILTER_MUTEX_TIMEOUT_MS)) != pdTRUE)
        {
            LOG_ERROR("[propagate] Timed out waiting for filter mutex");
            continue;
        }
        slate.attitude_filter.compute_dynamics_matrix(slate.imu_data.w_body, dt);
        slate.attitude_filter.propagate_covariance();
        slate.attitude_filter.progagate_attitude(dt);
        xSemaphoreGive(slate.filter_mutex);

        LOG_INFO("[propagate] q = [%f, %f, %f, %f]",
                slate.attitude_filter.quat_.x,
                slate.attitude_filter.quat_.y,
                slate.attitude_filter.quat_.z,
                slate.attitude_filter.quat_.w
                );
    }
}

void vTaskResetEstimate(void *) {
    stdio_flush();
    for(;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_RESET_ESTIMATE))
        TASK_LOOP_MS(200) // 5 Hz

        LOG_INFO("[error reset] bias estimate: [%f, %f, %f]",
                slate.attitude_filter.bias_estimate_.x,
                slate.attitude_filter.bias_estimate_.y,
                slate.attitude_filter.bias_estimate_.z
                );
        LOG_INFO("[error reset] w_body: [%f, %f, %f]",
                slate.imu_data.w_body.x,
                slate.imu_data.w_body.y,
                slate.imu_data.w_body.z
                );

        if (xSemaphoreTake(slate.filter_mutex,
                           pdMS_TO_TICKS(FILTER_MUTEX_TIMEOUT_MS)) != pdTRUE)
        {
            LOG_ERROR("[error reset] Timed out waiting for filter mutex");
            continue;
        }
        slate.attitude_filter.reset_error();
        xSemaphoreGive(slate.filter_mutex);
    }
}
