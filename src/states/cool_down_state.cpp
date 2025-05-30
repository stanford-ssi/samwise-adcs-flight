/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines the emergency cool down state: it reads the sensors but
 * does nothing.
 */

#include "cool_down_state.h"

sched_state_t *cool_down_get_next_state(slate_t *slate)
{
    // This state is currently unused and is a no-op
    return &cool_down_state;
}

sched_state_t cool_down_state = {.name = "cool_down_state",
                                 .num_tasks = 0,
                                 .task_list = {},
                                 .get_next_state = &cool_down_get_next_state};
