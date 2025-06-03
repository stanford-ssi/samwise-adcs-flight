/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines the types used in the state machine
 *
 */
#pragma once

#include "pico/types.h"

#define MAX_TASKS_PER_STATE 10

/*
 * Forward declare the slate type since it depends on us
 */
typedef struct samwise_adcs_slate slate_t;

/**
 * Holds the info for a single task. A single task can belong to multiple
 * states.
 */
typedef struct sched_task
{
    /**
     * Friendly name for the task.
     */
    const char *name;

    /**
     * Minimum number of milliseconds between dispatches of this task.
     */
    const uint32_t dispatch_period_ms;

    /**
     * Called once when the task initializes.
     * @param slate     Pointer to the current satellite slate
     */
    void (*task_init)(slate_t *slate);

    /**
     * Called each time the task dispatches.
     * @param slate     Pointer to the current satellite slate
     */
    void (*task_dispatch)(slate_t *slate);

    /**
     * Earliest time this task can be dispatched.
     */
    absolute_time_t next_dispatch;

} sched_task_t;

/**
 * Holds the info for defining a state.
 */
typedef struct sched_state
{
    /**
     * Friendly name for the state.
     */
    const char *name;

    size_t num_tasks;
    sched_task_t *task_list[MAX_TASKS_PER_STATE];

    /**
     * Called each time the state dispatches.
     * @param slate     Pointer to the current satellite slate
     * @return The next state to transition to
     */
    struct sched_state *(*get_next_state)(slate_t *slate);
} sched_state_t;