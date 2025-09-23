/**
 * @author Niklas Vainio
 * @date 2025-05-24
 *
 * This file contains functions for interacting with the PiCubed over UART
 */
#pragma once

#include "adcs_packet.h"
#include "slate.h"

void picubed_uart_init();

bool picubed_uart_handle_commands(slate_t *slate);