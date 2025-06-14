/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines a state for detumbling
 */

#include "detumble_state.h"

#include "constants.h"
#include "tasks/bdot_task.h"
#include "tasks/sensors_task.h"
#include "tasks/telemetry_task.h"

#include "states/cool_down_state.h"
#include "states/slewing_state.h"

sched_state_t *detumble_get_next_state(slate_t *slate)
{
    // Enter slewing at low angular velocity
    if (slate->imu_data_valid && (slate->w_mag < W_EXIT_DETUMBLE_THRESHOLD))
    {
        return &slewing_state;
    }

    // Enter cool down at excessively high angular velocity
    if (slate->imu_data_valid && (slate->w_mag > W_COOL_DOWN_ENTER_THRESHOLD))
    {
        return &cool_down_state;
    }

    // For now, just stay in the state
    return &detumble_state;
}

sched_state_t detumble_state = {
    .name = "detumble",
    .num_tasks = 3,
    .task_list = {&sensors_task, &telemetry_task, &bdot_task},
    .get_next_state = &detumble_get_next_state};
