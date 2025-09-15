/**
 * @author  Niklas Vainio
 * @date    2025-05-29
 *
 * Task to run tests - feel free to hook new code in here
 */

#include "test_task.h"
#include "macros.h"
#include "pico/stdlib.h"
#include "tests/software/sun_sensor_to_vector.h"
#include "constants.h"

void test_task_init(slate_t *slate)
{
    LOG_INFO("[test] Initializing test task!");
}

void test_task_dispatch(slate_t *slate)
{
    LOG_INFO("[test] TEST TASK IS DISPATCHING");
    test_eclipse(slate);
}

sched_task_t test_task = {.name = "test",
                          .dispatch_period_ms = 10,
                          .task_init = &test_task_init,
                          .task_dispatch = &test_task_dispatch,

                          /* Set to an actual value on init */
                          .next_dispatch = 0};