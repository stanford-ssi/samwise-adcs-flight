/**
 * @author Lundeen Cahilly
 * @date 2025-10-24
 *
 * This task is responsible for sending telemetry to the motor board to run
 * the reaction wheels
 */

#include "reaction_wheel_task.h"
#include "drivers/motor_board/motor_board.h"
#include "macros.h"

/**
 * @brief Populate telemetry data in the slate. Currently uses placeholder
 * values.
 *
 * @param slate Pointer to the current satellite slate
 */
static void populate_tx(slate_t *slate)
{
    // Copy array data from slate to tx packet
    for (int i = 0; i < NUM_REACTION_WHEELS; i++)
    {
        slate->motor_telemetry_tx.reaction_wheels_enabled[i] =
            slate->reaction_wheels_enabled[i];
        slate->motor_telemetry_tx.w_reaction_wheels_requested[i] =
            slate->w_reaction_wheels_requested[i];
    }

    // Compute checksum
    slate->motor_telemetry_tx.checksum = 0;
    uint8_t *data_ptr = (uint8_t *)&slate->motor_telemetry_tx;
    for (size_t i = 0; i < sizeof(motor_packet_tx_t) - sizeof(uint32_t); i++)
    {
        slate->motor_telemetry_tx.checksum += data_ptr[i];
    }
}

/**
 * @brief Initialize reaction wheel task.
 *
 * @param slate Pointer to the current satellite slate
 */
void reaction_wheel_task_init(slate_t *slate)
{
    LOG_INFO("[rw_task] Initializing motor board UART...");

    motor_board_uart_init();

    LOG_INFO("[rw_task] Motor Board UART successfully initialized!");
}

/**
 * @brief Dispatch telemetry task. Populates telemetry data and handles
 * picubed commands.
 *
 * @param slate Pointer to the current satellite slate
 */
void reaction_wheel_task_dispatch(slate_t *slate)
{
    LOG_INFO("[rw_task] Reaction wheel task dispatching...");
    populate_tx(slate);
    motor_board_uart_transmit(slate);
}

sched_task_t reaction_wheel_task = {
    .name = "reaction_wheels",
    .dispatch_period_ms =
        20, // TODO: determine appropriate period. for now, we do 50hz updates
    .task_init = &reaction_wheel_task_init,
    .task_dispatch = &reaction_wheel_task_dispatch,
    /* Set to an actual value on init */
    .next_dispatch = 0};