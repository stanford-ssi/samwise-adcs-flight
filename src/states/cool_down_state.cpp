/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines the emergency cool down state: it reads the sensors but
 * does nothing.
 */

#include "cool_down_state.h"

#include "tasks/sensors_task.h"
#include "tasks/telemetry_task.h"

sched_state_t *cool_down_get_next_state(slate_t *slate)
{
    // This state is currently unused and is a no-op
    // TODO: Transition to detumble if angular velocity is low enough
    return &cool_down_state;
}

sched_state_t cool_down_state = {.name = "cool_down",
                                 .num_tasks = 2,
                                 .task_list = {&sensors_task, &telemetry_task},
                                 .get_next_state = &cool_down_get_next_state};
