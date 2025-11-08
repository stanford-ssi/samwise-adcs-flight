/**
 * @author Matthew Musson, Lundeen Cahilly
 * @date 2025-11-08
 *
 * This file defines our default safe state that reads sensors and does nothing else
 */

#include "safe_state.h"
#include "tasks/system/telemetry_task.h"
#include "tasks/system/watchdog_task.h"
#include "tasks/sensors/gps_world_task.h"
#include "tasks/sensors/magnetometer_task.h"
#include "tasks/sensors/sun_sensor_task.h"
#include "tasks/sensors/power_monitor_task.h"

#include "tasks/sensors/imu_task.h"


#include "macros.h"

#include "drivers/neopixel/neopixel.h"

sched_state_t *safe_get_next_state(slate_t *slate)
{
    neopixel_set_color_rgb(255, 255, 0); // Yellow for safe state
    // If testing, go straight to test. Otherwise, go to detumble
    // TODO: add state override into other states
    return &safe_state;
}

sched_state_t safe_state = {.name = "safe",
                                .num_tasks = 7,
                                .task_list = {&watchdog_task,
                                              &telemetry_task,
                                              &gps_world_task,
                                              &magnetometer_task,
                                              &sun_sensor_task,
                                              &imu_task,
                                              &power_monitor_task},
                                .get_next_state = &safe_get_next_state};

