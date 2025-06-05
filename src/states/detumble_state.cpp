/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines a state for detumbling
 */

#include "detumble_state.h"

#include "tasks/bdot_task.h"
#include "tasks/sensors_task.h"
#include "tasks/telemetry_task.h"

sched_state_t *detumble_get_next_state(slate_t *slate)
{
    // TODO: Make this enter slewing at low angular velocity
    // TODO: Make this enter cool down at excessively high angular velocity
    // For now, just stay in the state
    return &detumble_state;
}

sched_state_t detumble_state = {
    .name = "detumble",
    .num_tasks = 3,
    .task_list = {&sensors_task, &telemetry_task, &bdot_task},
    .get_next_state = &detumble_get_next_state};
