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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "linalg.h"
#include "pico/types.h"

#include "drivers/adm1176/adm1176.h"
#include "drivers/motor/motor.h"
#include "drivers/software_uart/software_uart.h"
#include "drivers/telemetry/uart_package.h"
#include "drivers/watchdog_motor/watchdog.h"

#include "apps/motor_app/state_machine.h"

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
    volatile motor_state_t motor_measured[4];

    adcs_to_motor_package_t rx_package;
    motor_to_adcs_package_t tx_package;

    StateId_t current_state;
    TaskHandle_t state_machine_handler;
    EventGroupHandle_t events;
    QueueHandle_t state_queue_handle;

    int rx_count;

} motor_slate_t;

// extern motor_slate_t motor_slate;
