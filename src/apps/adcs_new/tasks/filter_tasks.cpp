#include "pico/time.h"

#include "macros.h"

#include "linalg.h"
using namespace linalg;
using namespace linalg::aliases;

#include "apps/adcs_new/tasks/tasks.h"
#include "apps/adcs_new/params.h"
#include "apps/adcs_new/slate.h"

#include "gnc/mekf/filter.h"

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
        LOG_DEBUG("[MAGNETOMETER FUSION] dt = [%d] ref = [%f, %f, %f]",
                dt,
                slate.b_field.b_eci.x,
                slate.b_field.b_eci.y,
                slate.b_field.b_eci.z);
       LOG_DEBUG("[MAGNETOMETER FUSION] observation = [%f, %f, %f]",
                slate.magnetometer_data.b_body.x,
                slate.magnetometer_data.b_body.y,
                slate.magnetometer_data.b_body.z
               );
        slate.magnetometer_fusion.set_reference(slate.b_field.b_eci);
        slate.magnetometer_fusion.compute_sensitivity(
                slate.attitude_filter.quat_);
        slate.magnetometer_fusion.compute_gain(
                slate.attitude_filter.covariance_mat_);
        slate.magnetometer_fusion.propagate_covariance_sensor(
                slate.attitude_filter.covariance_mat_);
        slate.magnetometer_fusion.apply_residual(
                slate.magnetometer_data.b_body, 
                slate.attitude_filter.error_estimate_);
    }
}

void vTaskSunVectorFusion(void *) {
    stdio_flush();
#ifdef GRAVITY_SENSOR
    float3 reference = {0, 0, 1};
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_SUN_VECTOR_FUSION));
        TASK_LOOP_MS(50); // 20 Hz
        if (!slate.imu_data.imu_accel_valid)
            continue;
        float3 a_obs = normalize(slate.imu_data.a_body);
        LOG_DEBUG("[GRAVITY FUSION] reference = [%f, %f, %f]",
                reference.x,
                reference.y,
                reference.z);
        LOG_DEBUG("[GRAVITY FUSION] observation = [%f, %f, %f]",
                a_obs.x,
                a_obs.y,
                a_obs.z);
        slate.magnetometer_fusion.set_reference(reference);
        slate.magnetometer_fusion.compute_sensitivity(
                slate.attitude_filter.quat_);
        slate.magnetometer_fusion.compute_gain(
                slate.attitude_filter.covariance_mat_);
        slate.magnetometer_fusion.propagate_covariance_sensor(
                slate.attitude_filter.covariance_mat_);
        slate.magnetometer_fusion.apply_residual(
                a_obs,
                slate.attitude_filter.error_estimate_);
 
    }
#endif
#ifndef GRAVITY_SENSOR
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_SUN_VECTOR_FUSION));
        TASK_LOOP_MS(50); // 20 Hz
        slate.magnetometer_fusion.set_reference(slate.sun_vector.sun_vector_eci);
        slate.magnetometer_fusion.compute_sensitivity(
                slate.attitude_filter.quat_);
        slate.magnetometer_fusion.compute_gain(
                slate.attitude_filter.covariance_mat_);
        slate.magnetometer_fusion.propagate_covariance_sensor(
                slate.attitude_filter.covariance_mat_);
        slate.magnetometer_fusion.apply_residual(
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


