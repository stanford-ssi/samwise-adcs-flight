/**
 * @author  Niklas Vainio
 * @date    2025-05-29
 *
 * Simple test task to print stuff
 */

#include "print_task.h"
#include "macros.h"

void print_task_init(slate_t *slate)
{
    LOG_INFO("Initializing print task!");
}

void print_task_dispatch(slate_t *slate)
{
    LOG_INFO("PRINTING!!!!");
}

sched_task_t print_task = {.name = "print",
                           .dispatch_period_ms = 1000,
                           .task_init = &print_task_init,
                           .task_dispatch = &print_task_dispatch,

                           /* Set to an actual value on init */
                           .next_dispatch = 0};
