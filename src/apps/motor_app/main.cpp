/**
**
* @author Carson Lauer
*
* This is the main motor file
*/
#include "pico/float.h"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "include/rtos_setup.h"

#include "hardware/gpio.h"

#include "macros.h"

#include "drivers/motor/motor.h"
#include "apps/motor_app/motor_tasks.h"
#include "apps/motor_app/init.h"
#include "apps/motor_app/motor_slate.h"
#include "apps/motor_app/pins.h"
#include "apps/motor_app/state_machine.h"

// Make sure top gpio bank enabled
static_assert(PICO_RP2350A == 0,
              "PICO_RP2350A must be defined to 0 for PICUBED builds.");

motor_slate_t motor_slate;

static TaskHandle_t task_handles[NTASKS];

static StaticTask_t telemetry_tcb;
static StackType_t  telemetry_stack[256];

static StaticTask_t power_tcb;
static StackType_t  power_stack[256];

static StaticTask_t watchdog_tcb;
static StackType_t  watchdog_stack[256];

static StaticTask_t control_tcb;
static StackType_t control_stack[256];

static StaticTask_t state_machine_tcb;
static StackType_t state_machine_stack[256];

int main() {

    stdio_init_all();
    sleep_ms(1000);

    // TODO: INIT SHOULD ALL BE IN A SEPARATE STATE IN THE STATE MACHINE
    // Initialize
    init(&motor_slate);

    // Initialize motors
    for (int m = 0; m < 4; m++)
    {
        motor_enable(&motor_slate.motors[m]);

        motor_slate.rx_package.target_rpm[m] = 2000.f;
        motor_slate.motor_state[m].rpm_ = 2000.f;

        motor_slate.motor_state[m].enabled_ = 1;
    }
    
    task_handles[TASK_TELEMETRY] = xTaskCreateStatic(
        telemetry_task,   // function
        "telemetry",      // name
        256,          // stack depth
        nullptr,      // params
        2,            // priority
        telemetry_stack,  // stack buffer
        &telemetry_tcb    // TCB buffer
    );

    task_handles[TASK_WATCHDOG] = xTaskCreateStatic(
        watchdog_feed_task,   // function
        "watchdog",      // name
        256,          // stack depth
        nullptr,      // params
        1,            // priority
        watchdog_stack,  // stack buffer
        &watchdog_tcb    // TCB buffer
    );

    task_handles[TASK_POWER] = xTaskCreateStatic(
        power_monitor_task,
        "power monitor",
        256,
        nullptr,
        1,
        power_stack,
        &power_tcb
    );

    task_handles[TASK_CONTROL] = xTaskCreateStatic(
        control_task,
        "control task",
        256,
        nullptr,
        1,
        control_stack,
        &control_tcb
    );

    motor_slate.state_machine_handler = xTaskCreateStatic(
        state_machine_task,
        "state machine orchestrator",
        256,
        nullptr,
        1,
        state_machine_stack,
        &state_machine_tcb
    );

    init_state_machine();

    vTaskStartScheduler();

    ERROR("END OF PROGRAM REACHED (BAD)");

    while (1)
        ;
}
