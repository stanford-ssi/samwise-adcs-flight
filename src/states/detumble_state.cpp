/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines a state for detumbling
 */

#include "detumble_state.h"

#include "params.h"
#include "tasks/control/actuators_task.h"
#include "tasks/control/bdot_task.h"
#include "tasks/navigation/reference_vector_task.h"
#include "tasks/sensors/gps_task.h"
#include "tasks/sensors/imu_task.h"
#include "tasks/sensors/magnetometer_task.h"
#include "tasks/sensors/power_monitor_task.h"
#include "tasks/sensors/sun_sensor_task.h"
#include "tasks/system/telemetry_task.h"
#include "tasks/system/watchdog_task.h"

#include "safe_state.h"

#include "drivers/neopixel/neopixel.h"

sched_state_t *detumble_get_next_state(slate_t *slate)
{
    neopixel_set_color_rgb(0, 0, 255); // Blue for detumble state

    // Enter safe state at low angular velocity OR if angular velocity is super
    // high (detumbled, await pointing mode)
    if (slate->imu_data_valid && ((slate->w_mag < W_EXIT_DETUMBLE_THRESHOLD) ||
                                  (slate->w_mag > W_ENTER_SAFE_THRESHOLD)))
    {
        return &safe_state;
    }

    // For now, just stay in the state
    return &detumble_state;
}

sched_state_t detumble_state = {
    .name = "detumble",
    .num_tasks = 9,
    .task_list = {&imu_task, &magnetometer_task, &sun_sensor_task,
                  &reference_vector_task, &gps_task, &power_monitor_task,
                  &telemetry_task, &bdot_task, &actuators_task, &watchdog_task},
    .get_next_state = &detumble_get_next_state};
