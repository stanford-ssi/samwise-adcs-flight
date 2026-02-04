#pragma once

#include "pico/stdlib.h"

#define BUFFER_SIZE 128

typedef struct
{
    uint32_t rx;
    uint32_t tx;
    uint32_t baud;
    uint32_t parity;
    // tx state machine
    uint32_t transmitting;
    uint32_t tx_alarm_id;
    uint32_t tx_bit;
    uint32_t tx_byte;
    // tx ring buffer
    uint8_t tx_buffer_start;
    uint8_t tx_buffer_end;
    uint8_t tx_buffer[BUFFER_SIZE];
    // rx state machine
    uint32_t rx_alarm_id;
    uint32_t receiving;
    uint32_t rx_bit;
    uint8_t rx_byte;
    // rx ring buffer
    uint8_t rx_buffer_start;
    uint8_t rx_buffer_end;
    uint8_t rx_buffer[BUFFER_SIZE];
} software_uart_t;

// Initializes software uart handler
software_uart_t software_uart_init(uint32_t rx, uint32_t tx);

// Initialize with more detailed settings
software_uart_t software_uart_init(uint32_t rx, uint32_t tx, uint32_t baud,
                                   uint32_t parity);

// Set the baud rate on a software uart handler
void software_uart_set_baud(software_uart_t *target, uint32_t baud);

// Set the parity on a software uart handler
void software_uart_set_parity(software_uart_t *target, uint32_t parity);

// Handle uart rx per bit
int64_t software_uart_handle_rx(alarm_id_t id, void *user_data);

// Handle uart rx start
void software_uart_handle_rx_start(software_uart_t *uart);

// Handle uart tx per bit
int64_t software_uart_handle_tx(alarm_id_t id, void *user_data);

// Handle uart tx start
void software_uart_handle_tx_start(software_uart_t *uart);

// Put char to end of tx buffer
void software_uart_tx_putbuf(software_uart_t *uart, uint8_t byte);

// check if uart i- readable
bool software_uart_is_readable(software_uart_t *uart);

// Get top of rx buffer
uint8_t software_uart_rx_getbuf(software_uart_t *uart);

// Handle uart tx start
void software_uart_putc(software_uart_t *uart, uint8_t c);

// Put null terminated string
void software_uart_putk(software_uart_t *uart, uint8_t *s);

template <typename T>
void software_uart_tx_package(software_uart_t *uart, T *p)
{
    char *q = (char *)p;
    for (int i = 0; i < sizeof(T); i++)
    {
        software_uart_tx_putbuf(uart, q[i]);
    }
    software_uart_handle_tx_start(uart);
}
