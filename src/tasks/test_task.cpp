/**
 * @author  Niklas Vainio
 * @date    2025-05-29
 *
 * Task to run tests - feel free to hook new code in here
 */

#include "macros.h"
#include "print_task.h"

void test_task_init(slate_t *slate)
{
    LOG_INFO("Initializing test task!");
}

void test_task_dispatch(slate_t *slate)
{
    LOG_INFO("TEST TASK IS DISPATCHING");
}

sched_task_t test_task = {.name = "print",
                          .dispatch_period_ms = 1000,
                          .task_init = &test_task_init,
                          .task_dispatch = &test_task_dispatch,

                          /* Set to an actual value on init */
                          .next_dispatch = 0};
