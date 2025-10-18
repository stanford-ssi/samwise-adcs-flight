/**
 * @author Niklas Vainio
 * @date 2025-05-24
 *
 * This file contains functions for interacting with the PiCubed over UART
 */

#include "hardware/flash.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#include "macros.h"
#include "picubed.h"
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

/**
 * @brief Helper function to read up to num_bytes bytes from ADCS uart with a
 * timeout in the case of missing bytes
 *
 * @param buf           Buffer to read into
 * @param num_bytes     Maximum number of bytes to read
 * @param timeout_us    Timeout between bytes in microseconds
 *
 * @return Number of bytes reac successfully (between 0 and num_bytes inclusive)
 */
static uint32_t read_uart_with_timeout(char *buf, uint32_t num_bytes,
                                       uint32_t timeout_us)
{
    for (uint32_t i = 0; i < num_bytes; i++)
    {
        if (uart_is_readable_within_us(SAMWISE_ADCS_PICUBED_UART, timeout_us))
        {
            buf[i] = uart_getc(SAMWISE_ADCS_PICUBED_UART);
        }
        else
        {
            return i;
        }
    }

    return num_bytes;
}

/**
 * Send a telemetry packet to the picubed over uart
 */
static void send_packet(const adcs_packet_t *packet)
{
    // log current telemetry packet
    LOG_DEBUG(
        "[picubed-uart] Sending telemetry packet: w=%.2f, q0=%.2f, q1=%.2f, "
        "q2=%.2f, q3=%.2f, state=%c, boot_count=%u",
        packet->w, packet->q0, packet->q1, packet->q2, packet->q3,
        packet->state, packet->boot_count);

    const char *data = (const char *)packet;

    // Log *data
    LOG_DEBUG("[picubed-uart] Sending telemetry packet data: %u",
              sizeof(adcs_packet_t));

    // Send bytes one by one
    for (uint32_t i = 0; i < sizeof(adcs_packet_t); i++)
    {
        printf("%02x ", (unsigned char)data[i]);
        uart_putc_raw(SAMWISE_ADCS_PICUBED_UART, data[i]);
    }
    printf("\n");
}

/**
 * Handle a command byte from the picubed.
 *
 * @param slate
 * @param cmd       Command byte to handle
 * @return True on success, false otherwise.
 */
static bool handle_command_byte(slate_t *slate, char command)
{
    LOG_INFO("[picubed-uart] Handling command byte %c", command);
    // sleep_ms(50); // Sleep for 20 milliseconds to simulate work

    switch (command)
    {
        case ADCS_SEND_TELEM:
            // Send telemetry
            send_packet(&slate->telemetry);
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
            // Erase a single flash sector at the provided offset
            uint32_t offset;
            uint32_t num_bytes = read_uart_with_timeout(
                (char *)offset, sizeof(offset), ADCS_BYTE_TIMEOUT_US);

            if (num_bytes < sizeof(offset))
            {
                LOG_ERROR(
                    "[picubed-uart] Bad flash erase command! Got %d/4 bytes",
                    num_bytes);
                return false;
            }

            LOG_INFO("[picubed-uart] Erasing flash offset 0x%x", offset);
            flash_range_erase(offset, 1);
            return true;
        }
        case ADCS_FLASH_PROGRAM:
        {
            // Program 256 bytes into flash at the desired offset
            uint32_t offset;
            uint32_t num_bytes = read_uart_with_timeout(
                (char *)offset, sizeof(offset), ADCS_BYTE_TIMEOUT_US);

            if (num_bytes < sizeof(offset))
            {
                LOG_ERROR(
                    "[picubed-uart] Bad flash program command! Got %d/4 bytes",
                    num_bytes);
                return false;
            }

            uint8_t data[256];
            num_bytes = read_uart_with_timeout((char *)data, sizeof(data),
                                               ADCS_BYTE_TIMEOUT_US);

            if (num_bytes < sizeof(offset))
                if (num_bytes < sizeof(data))
                {
                    LOG_ERROR("[picubed-uart] Bad flash program command! Got "
                              "%d/256 bytes",
                              num_bytes);
                    return false;
                }

            LOG_INFO("[picubed-uart] Programing flast at offset 0x%x", offset);
            flash_range_program(offset, data, sizeof(data));
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
 *
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
}

/**
 * Handle all available command packets from the picubed
 *
 * @return True on success, false otherwise
 */
bool picubed_uart_handle_commands(slate_t *slate)
{
    bool all_commands_succeeded = true;

    while (uart_is_readable(SAMWISE_ADCS_PICUBED_UART))
    {
        // Read byte and handle it
        char byte = uart_getc(SAMWISE_ADCS_PICUBED_UART);
        all_commands_succeeded &= handle_command_byte(slate, byte);
    }

    return all_commands_succeeded;
}
