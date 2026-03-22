/**
 * @author  Niklas Vainio
 * @date    2025-05-29
 *
 * This task is responsible for sending telemetry to the picubed. It runs in all
 * states.
 */

#include "telemetry_task.h"
#include "drivers/picubed/picubed.h"
#include "macros.h"

/**
 * @brief Populate telemetry data in the slate. Currently uses placeholder
 * values.
 *
 * @param slate Pointer to the current satellite slate
 * TODO: Replace with actual telemetry data
 */
static void populate_telemetry(slate_t *slate)
{
    // Keep this as an initializer list to ensure complete initialization
    slate->telemetry = {.w = 0.0f,
                        .q0 = 0.1f,
                        .q1 = 0.2f,
                        .q2 = 0.3f,
                        .q3 = 0.4f,
                        .state = 'X',
                        .boot_count = 42};
}

/**
 * @brief Initialize telemetry task. Initializes picubed UART.
 *
 * @param slate Pointer to the current satellite slate
 */
void telemetry_task_init(slate_t *slate)
{
    LOG_INFO("[telem] Initializing picubed UART...");

    picubed_uart_init(slate);

    LOG_INFO("[telem] Picubed UART successfully initialized!");
}

/**
 * @brief Dispatch telemetry task. Populates telemetry data and handles
 * picubed commands.
 *
 * @param slate Pointer to the current satellite slate
 */
void telemetry_task_dispatch(slate_t *slate)
{
    LOG_DEBUG("[telem] Telemetry task dispatching...");
    populate_telemetry(slate);
    picubed_uart_handle_commands(slate);
}

sched_task_t telemetry_task = {.name = "telem",
                               .dispatch_period_ms =
                                   100, // TODO: determine appropriate rate
                               .task_init = &telemetry_task_init,
                               .task_dispatch = &telemetry_task_dispatch,

                               /* Set to an actual value on init */
                               .next_dispatch = 0};
