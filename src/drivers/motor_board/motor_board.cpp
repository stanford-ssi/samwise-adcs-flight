/**
 * @author Lundeen Cahilly
 * @date 2025-10-24
 *
 * This file contains functions for interacting with the motor board over UART
 */

#include "hardware/uart.h"
#include "pico/stdlib.h"

#include "macros.h"
#include "motor_board.h"
#include "pins.h"

// UART parameters for motor board communication
#define MOTOR_BOARD_UART_BAUD (115200)
#define MOTOR_BOARD_UART_DATA_BITS (8)
#define MOTOR_BOARD_UART_STOP_BITS (1)
#define MOTOR_BOARD_UART_PARITY (UART_PARITY_NONE)

// Timeout for receiving bytes (microseconds)
#define MOTOR_BOARD_BYTE_TIMEOUT_US (100) // 0.1ms

/**
 * @brief Helper function to read bytes from motor board UART with timeout
 *
 * Uses polling (no interrupts) to read data from motor board
 *
 * @param buf           Buffer to read into
 * @param num_bytes     Number of bytes to read
 * @param timeout_us    Timeout between bytes in microseconds
 *
 * @return Number of bytes read successfully
 */
static uint32_t read_uart_with_timeout(uint8_t *buf, uint32_t num_bytes,
                                       uint32_t timeout_us)
{
    for (uint32_t i = 0; i < num_bytes; i++)
    {
        if (uart_is_readable_within_us(SAMWISE_ADCS_MOTOR_UART, timeout_us))
        {
            buf[i] = uart_getc(SAMWISE_ADCS_MOTOR_UART);
        }
        else
        {
            return i; // Timeout - return number of bytes read so far
        }
    }

    return num_bytes;
}

/**
 * @brief Verify checksum of received packet
 *
 * @param packet    Pointer to the packet to verify
 *
 * @return True if checksum is valid, false otherwise
 */
static bool verify_rx_checksum(const motor_packet_rx_t *packet)
{
    uint32_t calculated_checksum = 0;
    const uint8_t *data_ptr = (const uint8_t *)packet;

    // Calculate checksum over all bytes except the checksum field itself
    for (size_t i = 0; i < sizeof(motor_packet_rx_t) - sizeof(uint32_t); i++)
    {
        calculated_checksum += data_ptr[i];
    }

    return (calculated_checksum == packet->checksum);
}

/**
 * @brief Initialize motor board UART
 */
void motor_board_uart_init()
{
    // Initialize UART
    uart_init(SAMWISE_ADCS_MOTOR_UART, MOTOR_BOARD_UART_BAUD);

    // Set data format
    uart_set_format(SAMWISE_ADCS_MOTOR_UART, MOTOR_BOARD_UART_DATA_BITS,
                    MOTOR_BOARD_UART_STOP_BITS, MOTOR_BOARD_UART_PARITY);

    // Set up GPIO pins for UART
    gpio_set_function(SAMWISE_ADCS_TX_TO_MOTOR, GPIO_FUNC_UART);
    gpio_set_function(SAMWISE_ADCS_RX_FROM_MOTOR, GPIO_FUNC_UART);

    LOG_DEBUG("[motor-uart] UART initialized: baud=%d, TX=GPIO%d, RX=GPIO%d",
              MOTOR_BOARD_UART_BAUD, SAMWISE_ADCS_TX_TO_MOTOR,
              SAMWISE_ADCS_RX_FROM_MOTOR);
}

/**
 * @brief Receive telemetry from motor board (polling, no interrupts)
 *
 * Checks if data is available and reads a complete packet if present.
 *
 * @param slate     Pointer to the current satellite slate
 *
 * @return True if packet received and checksum valid, false otherwise
 */
bool motor_board_uart_receive(slate_t *slate)
{
    // Check if any data is available (non-blocking)
    if (!uart_is_readable(SAMWISE_ADCS_MOTOR_UART))
    {
        return false;
    }

    // Attempt to read full packet
    uint8_t *rx_buffer = (uint8_t *)&slate->motor_telemetry_rx;
    uint32_t bytes_read = read_uart_with_timeout(
        rx_buffer, sizeof(motor_packet_rx_t), MOTOR_BOARD_BYTE_TIMEOUT_US);

    // Check if we received a complete packet
    if (bytes_read != sizeof(motor_packet_rx_t))
    {
        LOG_DEBUG("[motor-uart] Incomplete packet received: %u/%u bytes",
                 bytes_read, sizeof(motor_packet_rx_t));
        return false;
    }

    // Verify checksum
    if (!verify_rx_checksum(&slate->motor_telemetry_rx))
    {
        LOG_ERROR("[motor-uart] Checksum verification failed");
        return false;
    }

    LOG_DEBUG("[motor-uart] Packet received successfully");
    return true;
}

/**
 * @brief Transmit telemetry to motor board
 *
 * Sends the motor_packet_tx_t from slate over UART.
 * Assumes the packet has already been populated with valid data and checksum.
 *
 * @param slate     Pointer to the current satellite slate
 *
 * @return True on success, false otherwise
 */
bool motor_board_uart_transmit(slate_t *slate)
{
    const uint8_t *tx_data = (const uint8_t *)&slate->motor_telemetry_tx;

    LOG_DEBUG("[motor-uart] Transmitting packet: size=%u bytes, checksum=0x%08x",
              sizeof(motor_packet_tx_t), slate->motor_telemetry_tx.checksum);

    // Send bytes one by one
    for (uint32_t i = 0; i < sizeof(motor_packet_tx_t); i++)
    {
        uart_putc_raw(SAMWISE_ADCS_MOTOR_UART, tx_data[i]);
    }

    return true;
}
