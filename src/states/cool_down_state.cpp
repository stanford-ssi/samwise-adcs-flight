/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines the emergency cool down state: it reads the sensors but
 * does nothing.
 */

#include "cool_down_state.h"

#include "tasks/sensors/gps_task.h"
#include "tasks/sensors/imu_task.h"
#include "tasks/sensors/magnetometer_task.h"
#include "tasks/sensors/power_monitor_task.h"
#include "tasks/navigation/reference_vector_task.h"
#include "tasks/sensors/sun_sensor_task.h"
#include "tasks/system/telemetry_task.h"
#include "tasks/system/watchdog_task.h"

#include "states/detumble_state.h"

#include "drivers/neopixel/neopixel.h"

sched_state_t *cool_down_get_next_state(slate_t *slate)
{
    neopixel_set_color_rgb(255, 255, 0); // Yellow for cool down state

    // Transition to detumble if angular velocity is low enough
    if (slate->imu_data_valid && (slate->w_mag < W_COOL_DOWN_EXIT_THRESHOLD))
    {
        return &detumble_state;
    }

    return &cool_down_state;
}

sched_state_t cool_down_state = {
    .name = "cool_down",
    .num_tasks = 8,
    .task_list = {&imu_task, &magnetometer_task, &sun_sensor_task,
                  &reference_vector_task, &gps_task, &power_monitor_task,
                  &telemetry_task, &watchdog_task},
    .get_next_state = &cool_down_get_next_state};
