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

#ifdef SIMULATION
#include "drivers/sim_interface/sim_interface.h"
#else
#include "drivers/magnetorquers/magnetorquers.h"
#endif

#include "pico/time.h"

void actuators_task_init(slate_t *slate)
{
    LOG_INFO("[actuators] Initializing actuators...");

#ifdef SIMULATION
    // Simulation mode - no hardware initialization needed
    LOG_INFO("[actuators] SIMULATION MODE - Actuator commands will be sent via USB");
#else
    // Initialize magnetorquer PWM
    LOG_INFO("[actuators] Initializing magnetorquer PWM...");
    init_magnetorquer_pwm();

    // TODO: Initialize reaction wheels
    LOG_INFO("[actuators] Reaction wheel control ready");
#endif

    // Initialize actuator request values to safe defaults
    slate->magdrv_requested = float3(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < NUM_REACTION_WHEELS; i++)
    {
        slate->reaction_wheels_w_requested[i] = 0.0f;
    }

    LOG_INFO("[actuators] Actuator initialization complete!");
}

void actuators_task_dispatch(slate_t *slate)
{
#ifdef SIMULATION
    // Simulation mode - send actuator commands to simulator via USB
    LOG_DEBUG("[actuators] Sending actuator commands - Mag: [%.3f, %.3f, %.3f], RW: [%.3f, %.3f, %.3f]",
              slate->magdrv_requested.x, slate->magdrv_requested.y, slate->magdrv_requested.z,
              slate->reaction_wheels_w_requested[0], slate->reaction_wheels_w_requested[1],
              slate->reaction_wheels_w_requested[2]);

    sim_send_actuators(slate);

    // Mark magnetorquers as "running" in simulation
    slate->magnetorquers_running = true;

    // Copy requested speeds to actual speeds (simulator handles the physics)
    for (int i = 0; i < NUM_REACTION_WHEELS; i++)
    {
        slate->reaction_wheel_speeds[i] = slate->reaction_wheels_w_requested[i];
    }
#else
    // Flight mode - drive real hardware
    // Drive magnetorquers based on slate requests
    bool mag_result = do_magnetorquer_pwm(slate->magdrv_requested);
    slate->magnetorquers_running = mag_result;

    if (!mag_result)
    {
        LOG_ERROR("[actuators] Magnetorquer PWM error");
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
#endif
}

sched_task_t actuators_task = {
    .name = "actuators",
    .dispatch_period_ms = 50, // Run at 20 Hz for responsive actuator control
    .task_init = &actuators_task_init,
    .task_dispatch = &actuators_task_dispatch,

    /* Set to an actual value on init */
    .next_dispatch = 0};