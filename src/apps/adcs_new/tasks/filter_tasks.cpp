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

#define GRAVITY_SENSOR

extern slate_t slate;

void vTaskMagnetometerFusion(void *) {
    absolute_time_t t_prev = get_absolute_time();
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_MAGNETOMETER_FUSION));
        TASK_LOOP_MS(100); // 10 Hz
        absolute_time_t t_now = get_absolute_time();
        uint32_t dt = absolute_time_diff_us(t_prev, t_now);
        t_prev = t_now;
        slate.magnetometer_fusion.set_reference(normalize(slate.b_field.b_enu));
        LOG_INFO("[MAGNETOMETER FUSION] expected = [%f, %f, %f]",
                slate.b_field.b_enu.x,
                slate.b_field.b_enu.y,
                slate.b_field.b_enu.z
                );
        slate.magnetometer_fusion.compute_sensitivity(
                slate.attitude_filter.quat_);
        slate.magnetometer_fusion.compute_gain(
                slate.attitude_filter.covariance_mat_);
        slate.magnetometer_fusion.propagate_covariance_sensor(
                slate.attitude_filter.covariance_mat_);
        slate.magnetometer_fusion.apply_residual(
                normalize(slate.magnetometer_data.b_body), 
                slate.attitude_filter.error_estimate_);
        slate.attitude_filter.reset_error();
        LOG_INFO("[MAGNETOMETER FUSION] expected = [%f, %f, %f]",
                slate.magnetometer_fusion.ref_body_.x,
                slate.magnetometer_fusion.ref_body_.y,
                slate.magnetometer_fusion.ref_body_.z
                );
        LOG_INFO("[MAGNETOMETER FUSION] observation = [%f, %f, %f]",
                slate.magnetometer_data.b_body.x,
                slate.magnetometer_data.b_body.y,
                slate.magnetometer_data.b_body.z
               );
    }
}

void vTaskSunVectorFusion(void *) {
#ifdef GRAVITY_SENSOR
    float3 reference = {0, 0, 1};
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_SUN_VECTOR_FUSION));
        TASK_LOOP_MS(50); // 20 Hz
        if (!slate.imu_data.imu_accel_valid)
            continue;
        // float3 lla = {slate.gps_data.gps_lat, 
        //     slate.gps_data.gps_lon, 
        //     slate.gps_data.gps_alt};
        // float3 ref_ecef = enu_to_ecef(reference, lla);
        // float3 ref_eci = ecef_to_eci(ref_ecef, slate.gps_data.MJD);
        // ref_eci = normalize(ref_eci);
        float3 a_obs = normalize(slate.imu_data.a_body);
        slate.sun_vector_fusion.set_reference(reference);
        slate.sun_vector_fusion.compute_sensitivity(
                slate.attitude_filter.quat_);
        slate.sun_vector_fusion.compute_gain(
                slate.attitude_filter.covariance_mat_);
        slate.sun_vector_fusion.propagate_covariance_sensor(
                slate.attitude_filter.covariance_mat_);
        slate.sun_vector_fusion.apply_residual(
                a_obs,
                slate.attitude_filter.error_estimate_);
        slate.attitude_filter.reset_error();
        LOG_INFO("[GRAVITY FUSION] expected = [%f, %f, %f]",
                slate.sun_vector_fusion.ref_body_.x,
                slate.sun_vector_fusion.ref_body_.y,
                slate.sun_vector_fusion.ref_body_.z
                );
        LOG_INFO("[GRAVITY FUSION] observation = [%f, %f, %f]",
                a_obs.x,
                a_obs.y,
                a_obs.z
               );
 
    }
#endif
#ifndef GRAVITY_SENSOR
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_SUN_VECTOR_FUSION));
        TASK_LOOP_MS(50); // 20 Hz
        slate.sun_vector_fusion.set_reference(slate.sun_vector.sun_vector_eci);
        slate.sun_vector_fusion.compute_sensitivity(
                slate.attitude_filter.quat_);
        slate.sun_vector_fusion.compute_gain(
                slate.attitude_filter.covariance_mat_);
        slate.sun_vector_fusion.propagate_covariance_sensor(
                slate.attitude_filter.covariance_mat_);
        slate.sun_vector_fusion.apply_residual(
                slate.sun_sensor.sun_vector_body, 
                slate.attitude_filter.error_estimate_);
 
    }
#endif
}

void vTaskPropagate(void *) {
    stdio_flush();
    for(;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_PROPAGATE));
        TASK_LOOP_MS(20); // 50 Hz

        // float3 w_fake = {0, 0, 2*PI*1.0f};
        slate.attitude_filter.compute_dynamics_matrix(
                slate.imu_data.w_body, 0.02f); 
        slate.attitude_filter.propagate_covariance();
        slate.attitude_filter.progagate_attitude(0.02f);
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
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_RESET_ESTIMATE));
        TASK_LOOP_MS(200); // 50 Hz
        LOG_INFO("[error reset] error reset bias: [%f, %f, %f]",
                slate.attitude_filter.bias_estimate_.x,
                slate.attitude_filter.bias_estimate_.y,
                slate.attitude_filter.bias_estimate_.z
                );
        slate.attitude_filter.reset_error();
    }
}


