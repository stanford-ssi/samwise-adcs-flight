#pragma once
#include "hardware/pio.h"
#include "stdint.h"

void pio_uart_comms_init(uint8_t tx, uint8_t rx, uint32_t baud);
uint16_t pio_uart_comms_tx(uint8_t *data, uint16_t length);
uint16_t pio_uart_comms_packet_ready();
uint16_t pio_uart_comms_get_packet(uint8_t *buffer, uint16_t max_length);
