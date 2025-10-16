/**
 * @author Lundeen Cahilly
 * @date 2025-08-13
 *
 * This task is responsible for feeding the watchdog timer to prevent system
 * resets. Should run in all states.
 */

#include "watchdog_task.h"
#include "drivers/neopixel/neopixel.h"
#include "drivers/watchdog/watchdog.h"
#include "macros.h"
#include "pico/time.h"
#include "slate.h"

/**
 * @brief Initialize watchdog task. Initializes the watchdog timer.
 *
 * @param slate Pointer to the current satellite slate
 */
void watchdog_task_init(slate_t *slate)
{
    watchdog_init(slate);
}

/**
 * @brief Dispatch watchdog task. Pets the watchdog timer to prevent system
 * resets.
 *
 * @param slate Pointer to the current satellite slate
 */
void watchdog_task_dispatch(slate_t *slate)
{
    neopixel_set_color_rgb(255, 0, 0); // Red blink for watchdog task
    watchdog_pet(slate);
    neopixel_set_color_rgb(0, 0, 0);
}

sched_task_t watchdog_task = {
    .name = "watchdog",
    .dispatch_period_ms =
        200, // Run frequently to handle both 20s timing and 200ms pulse
    .task_init = &watchdog_task_init,
    .task_dispatch = &watchdog_task_dispatch,

    .next_dispatch = 0};
