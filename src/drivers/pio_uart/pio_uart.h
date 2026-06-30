#pragma once
#include "hardware/pio.h"

void pio_uart_rx_init(PIO* pio_rx, uint *sm_rx, uint8_t pin, uint32_t baud);
void pio_uart_getc(PIO pio_rx, uint sm_rx);

void pio_uart_tx_init(PIO* pio, uint *sm_tx, uint8_t pin_tx, uint32_t baud);
void pio_uart_putc(PIO pio_tx, uint sm_tx, char c);
void pio_uart_puts(PIO pio_tx, uint sm_tx, const char* s);
