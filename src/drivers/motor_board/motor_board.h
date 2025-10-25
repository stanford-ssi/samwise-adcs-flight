/**
 * @author Lundeen Cahilly
 * @date 2025-10-24
 *
 * This file contains functions for interacting with the motor board over UART
 */
#pragma once

#include "packets.h"
#include "slate.h"

void motor_board_uart_init();

bool motor_board_uart_receive(slate_t *slate);

bool motor_board_uart_transmit(slate_t *slate);