#pragma once
#include "hardware/pio.h"

void pio_uart_rx_init(PIO* pio, uint8_t *sm, uint8_t pin, uint32_t baud);
void pio_uart_getc();

void pio_uart_tx_init(PIO* pio, uint8_t *sm, uint8_t pin_tx, uint32_t baud);
void pio_uart_putc(char c);
void pio_uart_puts(const char* s);
