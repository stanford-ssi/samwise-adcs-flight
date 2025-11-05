/**
    * @file emergency_state.cpp
 */

#include "emergency_state.h"
#include "tasks/sensors/power_monitor_task.h"
#include "tasks/system/watchdog_task.h"
#include "states/safe_state.h"

#include "macros.h"

#include "drivers/neopixel/neopixel.h"

sched_state_t *emergency_get_next_state(slate_t *slate)
{
    neopixel_set_color_rgb(255, 0, 0); // RED for emergency state
    // If testing, go straight to test. Otherwise, go to detumble
    if (slate->adcs_voltage > BATTERY_VOLTAGE_EMERGENCY)
    {
        return &safe_state;
    }
    return &emergency_state;
}

sched_state_t emergency_state = {.name = "emergency",
                                .num_tasks = 2,
                                .task_list = {&watchdog_task,
                                              &power_monitor_task},
                                .get_next_state = &emergency_get_next_state};
