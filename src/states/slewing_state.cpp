/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines a state for attitude slewing.
 */

#include "slewing_state.h"

#include "tasks/sensors/gps_world_task.h"
#include "tasks/sensors/imu_task.h"
#include "tasks/sensors/magnetometer_task.h"
#include "tasks/sensors/power_monitor_task.h"
#include "tasks/sensors/sun_sensor_task.h"
#include "tasks/system/telemetry_task.h"
#include "tasks/system/watchdog_task.h"

#include "states/detumble_state.h"

#include "drivers/neopixel/neopixel.h"

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
                               .num_tasks = 7,
                               .task_list = {&imu_task, &magnetometer_task,
                                             &sun_sensor_task, &gps_world_task,
                                             &power_monitor_task,
                                             &telemetry_task, &watchdog_task},
                               .get_next_state = &slewing_get_next_state};
