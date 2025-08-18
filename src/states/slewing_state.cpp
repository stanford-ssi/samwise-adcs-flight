/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines a state for attitude slewing.
 */

#include "slewing_state.h"

#include "tasks/sensors_task.h"
#include "tasks/telemetry_task.h"

#include "states/detumble_state.h"

#include "../drivers/neopixel.h"

sched_state_t *slewing_get_next_state(slate_t *slate)
{
    neopixel_set_color_rgb(255, 192, 203); // Pink for slewing state
    // This state is currently a no-op and unused

    // If angular speed is too high, go back to detumble
    if (slate->imu_data_valid && (slate->w_mag > W_ENTER_DETUMBLE_THRESHOLD))
    {
        return &detumble_state;
    }

    return &slewing_state;
}

sched_state_t slewing_state = {.name = "slewing",
                               .num_tasks = 2,
                               .task_list = {&sensors_task, &telemetry_task},
                               .get_next_state = &slewing_get_next_state};
