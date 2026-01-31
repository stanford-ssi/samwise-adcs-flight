/**
 * @author  The ADCS Team :)
 * @date    2024-02-08
 *
 * This file defines the slate struct, a static struct which stores all data on
 * the satellite. At init time, a single instance of this struct gets statically
 * allocated, and it is referenced by all tasks and functions.
 *
 * Look up "blackboard pattern" for more info.
 */

#pragma once

#include "linalg.h"
#include "pico/types.h"

#include "drivers/adm1176/adm1176.h"
#include "drivers/software_uart/software_uart.h"
#include "drivers/watchdog_motor/watchdog.h"
#include "drivers/motor/motor.h"
#include "drivers/telemetry/uart_package.h"

using namespace linalg::aliases;
using namespace linalg;

typedef struct 
{
	watchdog_t watchdog;
	adm1176_t power_monitor;

    software_uart_t adcs_uart;

    float voltage;
    float current;

	motor_t motors[4];
    motor_state_t motor_state[4];
    volatile motor_state_t  motor_measured[4];

    struct repeating_timer control_timer;
    struct repeating_timer telem_timer;

    adcs_to_motor_package_t rx_package;
    motor_to_adcs_package_t tx_package;

    int rx_count;

} motor_slate_t;

extern motor_slate_t motor_slate;
