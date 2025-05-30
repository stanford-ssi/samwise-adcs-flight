/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines the state that we first boot into
 */

#include "init_state.h"
#include "detumble_state.h"
#include "test_state.h"

#include "macros.h"

sched_state_t *init_get_next_state(slate_t *slate)
{
    // If testing, go straight to test. Otherwise, go to detumble
    if (IS_TEST)
    {
        return &test_state;
    }
    else
    {
        return &detumble_state;
    }
}

sched_state_t init_state = {.name = "init",
                            .num_tasks = 0,
                            .task_list = {},
                            .get_next_state = &init_get_next_state};
