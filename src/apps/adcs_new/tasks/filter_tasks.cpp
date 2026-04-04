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
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_SUN_VECTOR_FUSION));
        TASK_LOOP_MS(100); // 10 Hz
        absolute_time_t t_now = get_absolute_time();
        uint32_t dt = absolute_time_diff_us(t_prev, t_now);
        t_prev = t_now;
        LOG_DEBUG("[MAGNETOMETER FUSION] dt = [%d] ref = [%f, %f, %f]",
                dt,
                slate.b_field.b_eci.x,
                slate.b_field.b_eci.y,
                slate.b_field.b_eci.z);
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
#ifdef GRAVITY_SENSOR
    float3 reference = {0, 0, 1};
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_SUN_VECTOR_FUSION));
        TASK_LOOP_MS(50); // 20 Hz
        LOG_DEBUG("[GRAVITY FUSION] ref = [%f, %f, %f]",
                reference.x,
                reference.y,
                reference.z);
        slate.magnetometer_fusion.set_reference(reference);
        slate.magnetometer_fusion.compute_sensitivity(
                slate.attitude_filter.quat_);
        slate.magnetometer_fusion.compute_gain(
                slate.attitude_filter.covariance_mat_);
        slate.magnetometer_fusion.propagate_covariance_sensor(
                slate.attitude_filter.covariance_mat_);
        slate.magnetometer_fusion.apply_residual(
                normalize(slate.imu_data.a_body),
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
    for(;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_PROPAGATE));
        TASK_LOOP_MS(20); // 50 Hz
        slate.attitude_filter.compute_dynamics_matrix(
                slate.imu_data.w_body, 0.02f); 
        slate.attitude_filter.propagate_covariance();
        slate.attitude_filter.progagate_attitude(0.02f);
    }
}

void vTaskResetEstimate(void *) { 
    for(;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_RESET_ESTIMATE));
        TASK_LOOP_MS(200); // 50 Hz
        LOG_DEBUG("[progagator] error reset bias: [%f, %f, %f]",
                slate.attitude_filter.bias_estimate_.x,
                slate.attitude_filter.bias_estimate_.y,
                slate.attitude_filter.bias_estimate_.z
                );
        slate.attitude_filter.reset_error();
    }
}


