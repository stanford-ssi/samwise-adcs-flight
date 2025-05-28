/**
 * @author Niklas Vainio
 * @date 2025-05-24
 *
 * This file contains functions for sending telemetry over UART to the picubed
 */
#pragma once

#include "adcs_packet.h"
#include "slate.h"

void picubed_uart_init();

void picubed_uart_send_packet(const adcs_packet_t *packet);

// TO BE RUN ON THE PICUBED

void adcs_telemetry_task_init(slate_t *slate);

void adcs_telemetry_task_dispatch(slate_t *slate);
