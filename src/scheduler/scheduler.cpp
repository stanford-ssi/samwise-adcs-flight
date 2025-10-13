/**
 * @author  Niklas Vainio
 * @date    2024-08-25
 *
 * This file defines the tasks and states of the state machine, used for
 * scheduling and dispatching tasks on the satellite.
 */

#include "scheduler.h"

// Must include declarations of all states
#include "states/cool_down_state.h"
#include "states/detumble_state.h"
#include "states/init_state.h"
#include "states/slewing_state.h"
#include "states/test_state.h"

#ifdef SIMULATION
#include "drivers/sim_interface/sim_interface.h"
#endif

/*
 * Include the actual state machine
 */
static const sched_state_t *all_states[] = {&init_state, &test_state,
                                            &detumble_state, &slewing_state,
                                            &cool_down_state};
static sched_state_t *const initial_state = &init_state;

static size_t n_tasks = 0;
static sched_task_t *all_tasks[num_states * MAX_TASKS_PER_STATE];

/**
 * Initialize the state machine.
 *
 * @param slate     Pointer to the slate.
 */
void sched_init(slate_t *slate)
{
    /*
     * Check that each state has a valid number of tasks, and enumerate all
     * tasks.
     */
    for (size_t i = 0; i < num_states; i++)
    {
        ASSERT(all_states[i]->num_tasks <= MAX_TASKS_PER_STATE);
        for (size_t j = 0; j < all_states[i]->num_tasks; j++)
        {
            bool is_duplicate = 0;
            for (size_t k = 0; k < n_tasks; k++)
            {
                if (all_tasks[k] == all_states[i]->task_list[j])
                    is_duplicate = 1;
            }
            if (!is_duplicate)
            {
                all_tasks[n_tasks] = all_states[i]->task_list[j];
                n_tasks++;
            }
        }
    }

    LOG_DEBUG("sched: Enumerated %d tasks", n_tasks);

    /*
     * Initialize all tasks.
     */
    for (size_t i = 0; i < n_tasks; i++)
    {
        LOG_DEBUG("sched: Initializing task %s", all_tasks[i]->name);
        all_tasks[i]->task_init(slate);
    }

    for (size_t i = 0; i < n_tasks; i++)
    {
#ifdef SIMULATION
        all_tasks[i]->next_dispatch =
            sim_make_timeout_time_ms(slate, all_tasks[i]->dispatch_period_ms);
#else
        all_tasks[i]->next_dispatch =
            make_timeout_time_ms(all_tasks[i]->dispatch_period_ms);
#endif
    }

    /*
     * Enter the init state by default
     */
    slate->current_state = initial_state;
#ifdef SIMULATION
    slate->entered_current_state_time = sim_get_absolute_time(slate);
#else
    slate->entered_current_state_time = get_absolute_time();
#endif
    slate->time_in_current_state_ms = 0;

    LOG_DEBUG("sched: Done initializing!");
}

/**
 * Dispatch the state machine. Runs any of the current state's tasks which are
 * due, and transitions into the next state.
 *
 * @param slate     Pointer to the slate.
 */
void sched_dispatch(slate_t *slate)
{
    sched_state_t *current_state_info = slate->current_state;

#ifdef SIMULATION
    // Simulation mode: Run tasks sequentially without time checks
    // Flow: sensors (blocks) -> other tasks -> loop back

    static size_t task_index = 0;

    // If starting a new cycle, reset to first task
    if (task_index >= current_state_info->num_tasks)
    {
        task_index = 0;
    }

    // Execute current task
    if (task_index < current_state_info->num_tasks)
    {
        sched_task_t *task = current_state_info->task_list[task_index];
        LOG_DEBUG("[sched] Dispatching task: %s (index %zu/%zu)",
                  task->name, task_index, current_state_info->num_tasks - 1);
        task->task_dispatch(slate);
        task_index++;
    }

    // If we've run all tasks, check for state transition
    if (task_index >= current_state_info->num_tasks)
    {
        absolute_time_t current_time = sim_get_absolute_time(slate);
        slate->time_in_current_state_ms =
            absolute_time_diff_us(slate->entered_current_state_time, current_time) / 1000;

        sched_state_t *const next_state = current_state_info->get_next_state(slate);
        if (next_state != current_state_info)
        {
            LOG_INFO("sched: State transition: %s -> %s",
                     current_state_info->name, next_state->name);

            slate->current_state = next_state;
            slate->entered_current_state_time = current_time;
            slate->time_in_current_state_ms = 0;
            task_index = 0;  // Reset for new state
        }
    }
#else
    // Flight mode: Time-based scheduling
    absolute_time_t current_time = get_absolute_time();

    /*
     * Loop through all of this state's tasks
     */
    for (size_t i = 0; i < current_state_info->num_tasks; i++)
    {
        sched_task_t *task = current_state_info->task_list[i];

        /*
         * Check if this task is due and if so, dispatch it
         */
        if (absolute_time_diff_us(task->next_dispatch, current_time) > 0)
        {
            task->next_dispatch = make_timeout_time_ms(task->dispatch_period_ms);
            task->task_dispatch(slate);
        }
    }

    slate->time_in_current_state_ms =
        absolute_time_diff_us(slate->entered_current_state_time, current_time) / 1000;

    /*
     * Transition to the next state, if required.
     */
    sched_state_t *const next_state = current_state_info->get_next_state(slate);
    if (next_state != current_state_info)
    {
        LOG_INFO("sched: State transition: %s -> %s",
                 current_state_info->name, next_state->name);

        slate->current_state = next_state;
        slate->entered_current_state_time = current_time;
        slate->time_in_current_state_ms = 0;
    }
#endif
}
