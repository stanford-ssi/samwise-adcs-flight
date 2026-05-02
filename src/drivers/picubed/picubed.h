/**
 * @author Niklas Vainio
 * @date 2025-05-24
 *
 * This file contains functions for interacting with the PiCubed over UART
 */
#pragma once

#include "adcs_packet.h"

#include "drivers/communications/cobs.h"
#include "drivers/communications/protocol.h"
#include "drivers/communications/uart_communications.h"

void picubed_uart_init();

bool picubed_uart_handle_commands();

void receive_msg(msg_t *msg, uint8_t *rx_buf);

void send_msg(msg_t *msg, uint32_t len); 

void send_ping();

void send_pong();
