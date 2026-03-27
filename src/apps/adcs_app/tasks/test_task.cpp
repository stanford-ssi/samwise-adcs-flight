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
}

sched_task_t test_task = {.name = "test",
                          .dispatch_period_ms = 1000,
                          .task_init = &test_task_init,
                          .task_dispatch = &test_task_dispatch,

                          /* Set to an actual value on init */
                          .next_dispatch = 0};