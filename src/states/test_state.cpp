/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines a test state - we onlt enter this state if the TEST symbol
 * is defined.
 */

#include "test_state.h"
#include "tasks/system/watchdog_task.h"
#include "tasks/test_task.h"
#include "tasks/sensing/sensors_task.h"
#include "tasks/control/actuators_task.h"
#include "tasks/control/bdot_task.h"

#include "drivers/neopixel/neopixel.h"

sched_state_t *test_get_next_state(slate_t *slate)
{
    // Stay in the test state forever
    neopixel_set_color_rgb(128, 0, 128); // Purple for test state
    return &test_state;
}

// Add test tasks to the task list
sched_state_t test_state = {.name = "test",
                            .num_tasks = 4,
                            .task_list = {&watchdog_task, &sensors_task, &actuators_task, &bdot_task},
                            .get_next_state = &test_get_next_state};
