/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines a state for detumbling
 */

#include "detumble_state.h"
#include "tasks/print_task.h"

sched_state_t *detumble_get_next_state(slate_t *slate)
{
    // TODO: Make this enter slewing
    // For now, just stay in the state
    return &detumble_state;
}

sched_state_t detumble_state = {.name = "detumble",
                                .num_tasks = 1,
                                .task_list = {&print_task},
                                .get_next_state = &detumble_get_next_state};
