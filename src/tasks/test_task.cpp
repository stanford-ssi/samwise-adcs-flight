/**
 * @author  Niklas Vainio
 * @date    2025-05-29
 *
 * Task to run tests - feel free to hook new code in here
 * NOTE: please do not actually use git tracking on this file, as it is just for
 * quick tests and experiments
 */

#include "test_task.h"
#include "macros.h"
#include "pico/stdlib.h"

/**
 * @brief Initialize test task. Currently does nothing.
 *
 * @param slate Pointer to the current satellite slate
 */
void test_task_init(slate_t *slate)
{
    LOG_INFO("[test] Initializing test task!");
}

/**
 * @brief Dispatch test task. Currently just logs a message.
 *
 * @param slate Pointer to the current satellite slate
 */
void test_task_dispatch(slate_t *slate)
{
    LOG_INFO("[test] TEST TASK IS DISPATCHING");

    // Populate with dummy data
    slate->reaction_wheels_enabled[0] = true;
    slate->reaction_wheels_enabled[1] = true;
    slate->reaction_wheels_enabled[2] = true;
    slate->reaction_wheels_enabled[3] = true;
    slate->w_reaction_wheels_requested[0] = 1000.0f;
    slate->w_reaction_wheels_requested[1] = 3500.0f;
    slate->w_reaction_wheels_requested[2] = 1000.0f;
    slate->w_reaction_wheels_requested[3] = 1000.0f;

    // log telemetry packet
    LOG_INFO("[test] Reaction Wheel Telemetry:");
    LOG_INFO("  Checksum: %u", slate->motor_telemetry_tx.checksum);
    LOG_INFO("  RW0 Enabled: %d, Requested Speed: %.2f rad/s",
             slate->motor_telemetry_tx.reaction_wheels_enabled[0],
             slate->motor_telemetry_tx.w_reaction_wheels_requested[0]);

    LOG_INFO("  RW1 Enabled: %d, Requested Speed: %.2f rad/s",
             slate->motor_telemetry_tx.reaction_wheels_enabled[1],
             slate->motor_telemetry_tx.w_reaction_wheels_requested[1]);

    LOG_INFO("  RW2 Enabled: %d, Requested Speed: %.2f rad/s",
             slate->motor_telemetry_tx.reaction_wheels_enabled[2],
             slate->motor_telemetry_tx.w_reaction_wheels_requested[2]);

    LOG_INFO("  RW3 Enabled: %d, Requested Speed: %.2f rad/s",
             slate->motor_telemetry_tx.reaction_wheels_enabled[3],
             slate->motor_telemetry_tx.w_reaction_wheels_requested[3]);
}

sched_task_t test_task = {.name = "test",
                          .dispatch_period_ms = 1000,
                          .task_init = &test_task_init,
                          .task_dispatch = &test_task_dispatch,

                          /* Set to an actual value on init */
                          .next_dispatch = 0};