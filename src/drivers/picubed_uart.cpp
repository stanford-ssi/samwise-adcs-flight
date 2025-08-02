/**
 * @author Niklas Vainio
 * @date 2025-05-24
 *
 * This file contains functions for interacting with the PiCubed over UART
 */

#include "hardware/flash.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <string.h>

#include "macros.h"
#include "picubed_uart.h"
#include "pins.h"

// Uart parameters
#define PICUBED_UART_BAUD (115200)
#define PICUBED_UART_DATA_BITS (8)
#define PICUBED_UART_STOP_BITS (1)
#define PICUBED_UART_PARITY (UART_PARITY_NONE)

// Sentinel bytes for commands
// IMPORTANT: Keep in sync with adcs_driver.c on the picubed
#define ADCS_SEND_TELEM ('T')
#define ADCS_HEALTH_CHECK ('?')
#define ADCS_HEALTH_CHECK_SUCCESS ('!')

// Backdoor for flash programming to enable potential OTA in the future
#define ADCS_FLASH_ERASE ('E')
#define ADCS_FLASH_PROGRAM ('P')

// Timeout between bytes in microseconds
// currently set to 1 second
#define ADCS_BYTE_TIMEOUT_US (1000000)

// Command buffer for interrupt-driven reception (similar to GPS buffer)
#define PICUBED_CMD_BUFFER_SIZE (260) // Max 256 bytes data + 4 byte offset
static char picubed_cmd_buffer[PICUBED_CMD_BUFFER_SIZE];
static volatile uint16_t cmd_buffer_index = 0;
static volatile bool command_ready = false;
static volatile char current_command = 0;
static volatile bool expecting_flash_data = false;
static volatile uint16_t expected_bytes = 0;

/**
 * UART interrupt handler for PiCubed commands
 * Similar to GPS interrupt handler but for command processing
 */
void picubed_uart_irq_handler(void)
{
    LOG_DEBUG("[picubed-uart-irq] Interrupt triggered");

    while (uart_is_readable(SAMWISE_ADCS_PICUBED_UART))
    {
        char c = uart_getc(SAMWISE_ADCS_PICUBED_UART);
        LOG_DEBUG("[picubed-uart-irq] Received byte: '%c' (0x%02x)", c,
                  (uint8_t)c);
        LOG_DEBUG("[picubed-uart-irq] State: expecting_flash_data=%d, "
                  "command_ready=%d, current_command='%c', buffer_index=%d",
                  expecting_flash_data, command_ready, current_command,
                  cmd_buffer_index);

        // Handle simple single-byte commands
        if (!expecting_flash_data &&
            (c == ADCS_SEND_TELEM || c == ADCS_HEALTH_CHECK))
        {
            LOG_DEBUG("[picubed-uart-irq] Detected simple command: '%c'", c);
            // Only process if no command is pending
            if (!command_ready)
            {
                LOG_DEBUG(
                    "[picubed-uart-irq] Setting simple command ready: '%c'", c);
                current_command = c;
                command_ready = true;
                cmd_buffer_index = 0;
            }
            else
            {
                LOG_DEBUG("[picubed-uart-irq] Command already pending, "
                          "ignoring simple command '%c'",
                          c);
            }
        }
        // Handle flash commands that expect additional data
        else if (!expecting_flash_data &&
                 (c == ADCS_FLASH_ERASE || c == ADCS_FLASH_PROGRAM))
        {
            LOG_DEBUG("[picubed-uart-irq] Detected flash command: '%c'", c);
            if (!command_ready)
            {
                current_command = c;
                expecting_flash_data = true;
                expected_bytes = (c == ADCS_FLASH_ERASE)
                                     ? 4
                                     : 260; // 4 for offset, 260 for offset+data
                cmd_buffer_index = 0;
                LOG_DEBUG("[picubed-uart-irq] Set up flash command '%c', "
                          "expecting %d bytes",
                          c, expected_bytes);
            }
            else
            {
                LOG_DEBUG("[picubed-uart-irq] Command already pending, "
                          "ignoring flash command '%c'",
                          c);
            }
        }
        // Collect additional data for flash commands
        else if (expecting_flash_data &&
                 cmd_buffer_index < PICUBED_CMD_BUFFER_SIZE - 1)
        {
            LOG_DEBUG(
                "[picubed-uart-irq] Collecting flash data byte %d/%d: 0x%02x",
                cmd_buffer_index + 1, expected_bytes, (uint8_t)c);
            picubed_cmd_buffer[cmd_buffer_index] = c;
            cmd_buffer_index++;

            // Check if we have all expected bytes
            if (cmd_buffer_index >= expected_bytes)
            {
                LOG_DEBUG("[picubed-uart-irq] Flash command '%c' complete with "
                          "%d bytes",
                          current_command, cmd_buffer_index);
                command_ready = true;
                expecting_flash_data = false;
            }
        }
        else
        {
            // Invalid or unexpected data - reset state
            LOG_DEBUG("[picubed-uart-irq] Unexpected byte: '%c' (0x%02x), "
                      "resetting state",
                      c, (uint8_t)c);
            LOG_DEBUG("[picubed-uart-irq] Previous state: expecting_flash=%d, "
                      "ready=%d, cmd='%c', idx=%d",
                      expecting_flash_data, command_ready, current_command,
                      cmd_buffer_index);
            cmd_buffer_index = 0;
            command_ready = false;
            expecting_flash_data = false;
            current_command = 0;
        }
    }

    LOG_DEBUG("[picubed-uart-irq] Interrupt handler exit: ready=%d, cmd='%c', "
              "expecting_flash=%d, idx=%d",
              command_ready, current_command, expecting_flash_data,
              cmd_buffer_index);
}

/**
 * Send a telemetry packet to the picubed over uart
 */
static void send_packet(const adcs_packet_t *packet)
{
    const char *data = (const char *)packet;

    // Send bytes one by one
    for (uint32_t i = 0; i < sizeof(adcs_packet_t); i++)
    {
        uart_putc_raw(SAMWISE_ADCS_PICUBED_UART, data[i]);
    }
}

/**
 * Handle a complete command from the picubed using buffered data
 *
 * @param slate
 * @param command   Command byte to handle
 * @return True on success, false otherwise.
 */
static bool handle_complete_command(slate_t *slate, char command)
{
    LOG_INFO("[picubed-uart] Handling command byte %c", command);

    switch (command)
    {
        case ADCS_SEND_TELEM:
            // Send telemetry
            send_packet(&slate->telem);
            return true;
        case ADCS_HEALTH_CHECK:
            // Send a sentinel byte to report as healthy
            uart_putc_raw(SAMWISE_ADCS_PICUBED_UART, ADCS_HEALTH_CHECK_SUCCESS);
            return true;

            // *********************************************************************
            // The following section of code exists purely to provide a backdoor
            // for potential future OTA-ing of the ADCS board.
            //
            // DANGER: USING THIS INTERFACE INCORRECTLY COULD BRICK THE BOARD
            // PERMANENTLY. ONLY COMMAND THIS IF YOU ARE 100% SURE THAT IT WILL
            // WORK. IDEALLY THAT FIRST THING THAT EVER GETS PROGRAMMED HERE IS
            // A LESS DUMB OTA INTERFACE.

        case ADCS_FLASH_ERASE:
        {
            // Check we have the expected 4 bytes for offset
            if (cmd_buffer_index < 4)
            {
                LOG_ERROR(
                    "[picubed-uart] Bad flash erase command! Got %d/4 bytes",
                    cmd_buffer_index);
                return false;
            }

            // Extract offset from buffer (fix: use address, not value)
            uint32_t offset;
            memcpy(&offset, picubed_cmd_buffer, sizeof(offset));

            LOG_INFO("[picubed-uart] Erasing flash offset 0x%x", offset);
            flash_range_erase(offset, 1);
            return true;
        }
        case ADCS_FLASH_PROGRAM:
        {
            // Check we have the expected 260 bytes (4 offset + 256 data)
            if (cmd_buffer_index < 260)
            {
                LOG_ERROR("[picubed-uart] Bad flash program command! Got "
                          "%d/260 bytes",
                          cmd_buffer_index);
                return false;
            }

            // Extract offset from first 4 bytes (fix: use address, not value)
            uint32_t offset;
            memcpy(&offset, picubed_cmd_buffer, sizeof(offset));

            // Extract data from remaining 256 bytes
            uint8_t *data = (uint8_t *)(picubed_cmd_buffer + 4);

            LOG_INFO("[picubed-uart] Programming flash at offset 0x%x", offset);
            flash_range_program(offset, data, 256);
            return true;
        }
            // *********************************************************************

        default:
            // Invalid packet
            LOG_ERROR(
                "[picubed-uart] Encountered an invalid command byte: %c (0x%x)",
                command, command);
            return false;
    }
}

/**
 * Initialize uart for communication with picubed
 * Uses interrupt-driven reception like GPS driver
 */
void picubed_uart_init()
{
    // Initialize uart hardware
    uart_init(SAMWISE_ADCS_PICUBED_UART, PICUBED_UART_BAUD);

    gpio_init(SAMWISE_ADCS_TX_TO_PICUBED);
    gpio_set_function(SAMWISE_ADCS_TX_TO_PICUBED,
                      UART_FUNCSEL_NUM(SAMWISE_ADCS_PICUBED_UART,
                                       SAMWISE_ADCS_TX_TO_PICUBED));

    gpio_init(SAMWISE_ADCS_RX_FROM_PICUBED);
    gpio_set_function(SAMWISE_ADCS_RX_FROM_PICUBED,
                      UART_FUNCSEL_NUM(SAMWISE_ADCS_PICUBED_UART,
                                       SAMWISE_ADCS_RX_FROM_PICUBED));

    // Set data format
    uart_set_format(SAMWISE_ADCS_PICUBED_UART, PICUBED_UART_DATA_BITS,
                    PICUBED_UART_STOP_BITS, PICUBED_UART_PARITY);

    // Enable UART interrupt (similar to GPS driver)
    irq_set_exclusive_handler(UART_IRQ_NUM(SAMWISE_ADCS_PICUBED_UART),
                              picubed_uart_irq_handler);
    irq_set_enabled(UART_IRQ_NUM(SAMWISE_ADCS_PICUBED_UART), true);
    uart_set_irq_enables(SAMWISE_ADCS_PICUBED_UART, true,
                         false); // RX interrupt only

    // Initialize command buffer state
    cmd_buffer_index = 0;
    command_ready = false;
    current_command = 0;
    expecting_flash_data = false;
    expected_bytes = 0;

    LOG_INFO(
        "[picubed-uart] PiCubed UART initialized with interrupts at %d baud",
        PICUBED_UART_BAUD);
}

/**
 * Handle all available command packets from the picubed
 * Uses interrupt-buffered approach similar to GPS driver
 *
 * @return True on success, false otherwise
 */
bool picubed_uart_handle_commands(slate_t *slate)
{
    bool all_commands_succeeded = true;

    // Process any complete commands that arrived via interrupt
    if (command_ready)
    {
        LOG_DEBUG("[picubed-uart] Processing complete command: %c",
                  current_command);
        all_commands_succeeded &=
            handle_complete_command(slate, current_command);

        // Reset state for next command
        command_ready = false;
        current_command = 0;
        cmd_buffer_index = 0;
        expecting_flash_data = false;
        expected_bytes = 0;
    }

    return all_commands_succeeded;
}
