/**
 * @author Niklas Vainio
 * @date 2025-05-24
 *
 * This file contains functions for sending telemetry over UART to the picubed
 */

#include "hardware/uart.h"
#include "pico/stdlib.h"

#include "macros.h"
#include "picubed_uart.h"
#include "pins.h"

// Uart parameters
#define PICUBED_UART_BAUD (115200)
#define PICUBED_UART_DATA_BITS (8)
#define PICUBED_UART_STOP_BITS (1)
#define PICUBED_UART_PARITY (UART_PARITY_NONE)

// Software configuration for SLIP, a very simple packet framing technique
// Packets start with a start byte
// Start bytes in the message are escaped with ESC + ESC_START
// Escape bytes in the message are escaped with ESC + ESC_ESC
#define SLIP_START (0xC0)
#define SLIP_ESC (0xDB)
#define SLIP_ESC_START (0xDC)
#define SLIP_ESC_ESC (0xDD)

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
 * Send a telemetry packet to the picubed over uart
 */
void picubed_uart_send_packet(const adcs_packet_t *packet)
{
    const char *data = (const char *)packet;
    const uint32_t num_bytes = sizeof(adcs_packet_t);

    // Send start byte
    uart_putc_raw(SAMWISE_ADCS_PICUBED_UART, SLIP_START);

    // Encode each byte with SLIP
    for (uint32_t i = 0; i < num_bytes; i++)
    {
        switch (data[i])
        {
            case SLIP_START:
                // Escaped start byte
                uart_putc_raw(SAMWISE_ADCS_PICUBED_UART, SLIP_ESC);
                uart_putc_raw(SAMWISE_ADCS_PICUBED_UART, SLIP_ESC_START);
                break;
            case SLIP_ESC:
                // Escaped escape character
                uart_putc_raw(SAMWISE_ADCS_PICUBED_UART, SLIP_ESC);
                uart_putc_raw(SAMWISE_ADCS_PICUBED_UART, SLIP_ESC_ESC);
                break;
            default:
                // Normal byte
                uart_putc_raw(SAMWISE_ADCS_PICUBED_UART, data[i]);
                break;
        }
    }
}