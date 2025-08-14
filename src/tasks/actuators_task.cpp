/**
 * @author Lundeen Cahilly
 * @date 2025-08-14
 *
 * This task is responsible for driving the magnetorquers and reaction wheels
 * at the desired rates as indicated on the slate. It runs in all states where
 * actuators are needed.
 */

#include "actuators_task.h"
#include "macros.h"

#include "drivers/magnetorquer.h"
#include "pico/time.h"

void actuators_task_init(slate_t *slate)
{
    LOG_INFO("[actuators] Initializing actuators...");

    // Initialize magnetorquer PWM
    LOG_INFO("[actuators] Initializing magnetorquer PWM...");
    init_magnetorquer_pwm();

    // Initialize reaction wheels (no specific driver found, assuming they're
    // controlled via another interface)
    LOG_INFO("[actuators] Reaction wheel control ready");

    // Initialize actuator request values to safe defaults
    slate->magdrv_x_requested = 0.0f;
    slate->magdrv_y_requested = 0.0f;
    slate->magdrv_z_requested = 0.0f;

    for (int i = 0; i < NUM_REACTION_WHEELS; i++)
    {
        slate->reaction_wheels_w_requested[i] = 0.0f;
    }

    LOG_INFO("[actuators] Actuator initialization complete!");
}

void actuators_task_dispatch(slate_t *slate)
{
    // Drive magnetorquers based on slate requests
    // LOG_DEBUG("[actuators] Driving magnetorquers: X=%.3f, Y=%.3f, Z=%.3f",
    //           slate->magdrv_x_requested, slate->magdrv_y_requested,
    //           slate->magdrv_z_requested);

    // Convert from normalized (-1.0 to 1.0) to PWM duty cycle range (-128 to
    // 128)
    int8_t x_duty = (int8_t)(slate->magdrv_x_requested * PWM_MAX_DUTY_CYCLE);
    int8_t y_duty = (int8_t)(slate->magdrv_y_requested * PWM_MAX_DUTY_CYCLE);
    int8_t z_duty = (int8_t)(slate->magdrv_z_requested * PWM_MAX_DUTY_CYCLE);

    // Apply magnetorquer PWM with default max current limit
    pwm_error_t mag_result = do_magnetorquer_pwm(x_duty, y_duty, z_duty, 1000);

    if (mag_result != PWM_OK)
    {
        LOG_ERROR("[actuators] Magnetorquer PWM error: %d", mag_result);
    }

    // Drive reaction wheels based on slate requests
    // LOG_DEBUG("[actuators] Reaction wheel speeds requested: [%.3f, %.3f,
    // %.3f, %.3f] rad/s",
    //           slate->reaction_wheels_w_requested[0],
    //           slate->reaction_wheels_w_requested[1],
    //           slate->reaction_wheels_w_requested[2],
    //           slate->reaction_wheels_w_requested[3]);

    // TODO: Implement reaction wheel driver interface
    // For now, just copy the requested speeds to the actual speeds in the slate
    // This should be replaced with actual hardware driver calls
    for (int i = 0; i < NUM_REACTION_WHEELS; i++)
    {
        slate->reaction_wheel_speeds[i] = slate->reaction_wheels_w_requested[i];
    }
}

sched_task_t actuators_task = {
    .name = "actuators",
    .dispatch_period_ms = 50, // Run at 20 Hz for responsive actuator control
    .task_init = &actuators_task_init,
    .task_dispatch = &actuators_task_dispatch,

    /* Set to an actual value on init */
    .next_dispatch = 0};