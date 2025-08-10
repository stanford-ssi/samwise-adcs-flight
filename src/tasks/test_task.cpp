/**
 * @author  Niklas Vainio
 * @date    2025-05-29
 *
 * Task to run tests - feel free to hook new code in here
 */

#include "test_task.h"
#include "hardware/i2c.h"
#include "macros.h"
#include "sensors_task.h"
#include "telemetry_task.h"

void test_task_init(slate_t *slate)
{
    LOG_INFO("[test] Initializing test task!");
    sensors_task_init(slate);
    // telemetry_task_init(slate);
    // magnetorquer_tests_init();
}

void test_task_dispatch(slate_t *slate)
{
    LOG_INFO("[test] TEST TASK IS DISPATCHING");
    sensors_task_dispatch(slate);
    // telemetry_task_dispatch(slate);
    // magnetorquer_tests_dispatch();
}

sched_task_t test_task = {.name = "test",
                          .dispatch_period_ms = 10,
                          .task_init = &test_task_init,
                          .task_dispatch = &test_task_dispatch,

                          /* Set to an actual value on init */
                          .next_dispatch = 0};
