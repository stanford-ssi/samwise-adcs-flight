/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines a state for detumbling
 */

#include "detumble_state.h"

#include "constants.h"
#include "tasks/control/actuators_task.h"
#include "tasks/control/bdot_task.h"
#include "tasks/sensors/gps_world_task.h"
#include "tasks/sensors/imu_task.h"
#include "tasks/sensors/magnetometer_task.h"
#include "tasks/sensors/power_monitor_task.h"
#include "tasks/sensors/sun_sensor_task.h"
#include "tasks/system/telemetry_task.h"
#include "tasks/system/watchdog_task.h"

#include "states/cool_down_state.h"
#include "states/slewing_state.h"

#include "drivers/neopixel/neopixel.h"

sched_state_t *detumble_get_next_state(slate_t *slate)
{
    neopixel_set_color_rgb(0, 0, 255); // Blue for detumble state

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
    .num_tasks = 8,
    .task_list = {&imu_task, &magnetometer_task, &sun_sensor_task,
                  &gps_world_task, &power_monitor_task, &telemetry_task,
                  &bdot_task, &actuators_task, &watchdog_task},
    .get_next_state = &detumble_get_next_state};
