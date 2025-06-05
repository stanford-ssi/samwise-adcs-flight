/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines a test state - we onlt enter this state if the TEST symbol
 * is defined.
 */

#include "test_state.h"
#include "tasks/test_task.h"

sched_state_t *test_get_next_state(slate_t *slate)
{
    // Stay in the test state forever
    return &test_state;
}

// Add test tasks to the task list
sched_state_t test_state = {.name = "test",
                            .num_tasks = 1,
                            .task_list = {&test_task},
                            .get_next_state = &test_get_next_state};
