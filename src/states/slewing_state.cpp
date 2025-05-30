/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines a state for attitude slewing.
 */

#include "slewing_state.h"

sched_state_t *slewing_get_next_state(slate_t *slate)
{
    // This state is currently a no-op and unused
    return &slewing_state;
}

sched_state_t slewing_state = {.name = "slewing",
                               .num_tasks = 0,
                               .task_list = {},
                               .get_next_state = &slewing_get_next_state};
