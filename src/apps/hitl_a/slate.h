/*
 * Author:  @ Carson Lauer
 * Date :   @ January 24, 2025
 * Description:
 * Slate interface for hitl a board.
 * This board pretends to be adcs board.
 */

#pragma once

#include "linalg.h"
#include "pico/types.h"

#include "drivers/software_uart/software_uart.h"
#include "drivers/telemetry/uart_package.h"

using namespace linalg::aliases;
using namespace linalg;

typedef struct 
{
    telemetry_handler_t telemetry;
    software_uart_t adcs_uart;

    adcs_to_motor_package_t tx_package; // adcs ->
    motor_to_adcs_package_t rx_package; // adcs <-

    struct repeating_timer telem_timer;

    int rx_count;

} slate_t;

extern slate_t slate;
