#include "FreeRTOS.h"
#include "task.h"

#include "macros.h"

#include "pico/time.h"

#include "apps/adcs_new/params.h"
#include "apps/adcs_new/slate.h"
#include "apps/adcs_new/states/states.h"
#include "apps/adcs_new/tasks/tasks.h"
#include "apps/adcs_new/init.h"

#include "drivers/watchdog_motor/watchdog.h"

extern slate_t slate;
void vTaskInit(void *) {
    init_main();
    StateMsg_t msg = MSG_INIT_DONE;
    xQueueSend(slate.state_machine.state_queue_handle,
            &msg, 0);
    for(;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_INIT))
        TASK_LOOP_MS(100)
        LOG_ERROR("Reentered init function?");
    }
}

void vTaskWatchdog(void *) {
    // Edge-trigger the safing messages. The state queue is only
    // SENSOR_QUEUE_DEPTH deep, so repeating MSG_VOLTAGE_LOW every 100 ms would
    // push out real messages.
    bool voltage_low_latched = false;

    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_WATCHDOG))
        TASK_LOOP_MS(100)
        watchdog_feed(&slate.watchdog);

        // ALSO HANDLE POWER MONITOR
        if (!adm_get_power(&slate.power_monitor)) {
            LOG_ERROR("[power] Failed to read power monitor");
            // Fail closed on actuation: no power telemetry, no torquing.
            //
            // Deliberately do NOT safe the spacecraft here. A dead ADM1176 is
            // indistinguishable from a flat pack at this level, and safing on a
            // dead sensor would take the sensors and the estimator down with
            // it. Inhibiting the only load we control is the proportionate
            // response.
            slate.magtorq_armed = false;
            continue;
        }

        slate.power_read_time = get_absolute_time();

        LOG_INFO("[power] P = %.3fW, V = %.3fV, I = %.3fA",
                slate.power_monitor.power,
                slate.power_monitor.voltage,
                slate.power_monitor.current);

        const float voltage = slate.power_monitor.voltage;

        // Arm/disarm the magnetorquers with hysteresis, so the rods cannot
        // chatter as their own current draw sags the bus.
        if (slate.magtorq_armed && voltage < V_MAGTORQ_DISARM) {
            slate.magtorq_armed = false;
            LOG_ERROR("[power] Disarming magnetorquers at %.3f V", voltage);
        } else if (!slate.magtorq_armed && voltage > V_MAGTORQ_ARM) {
            slate.magtorq_armed = true;
            LOG_INFO("[power] Arming magnetorquers at %.3f V", voltage);
        }

        if (!voltage_low_latched && voltage < ADCS_VOLTAGE_UNSAFE) {
            voltage_low_latched = true;
            LOG_ERROR("[power] Bus at %.3f V, requesting safe mode", voltage);
            StateMsg_t msg = MSG_VOLTAGE_LOW;
            xQueueSend(slate.state_machine.state_queue_handle,
                    &msg, 0);
        } else if (voltage_low_latched && voltage > ADCS_VOLTAGE_SAFE_EXIT) {
            voltage_low_latched = false;
            LOG_INFO("[power] Bus recovered to %.3f V", voltage);
            StateMsg_t msg = MSG_VOLTAGE_OK;
            xQueueSend(slate.state_machine.state_queue_handle,
                    &msg, 0);
        }
    }
}

void vTaskStateMachine(void *) {
    enter_state(STATE_INIT);
    for(;;) {
        StateMsg_t msg; // Message
        xQueueReceive(slate.state_machine.state_queue_handle,
                &msg, 
                portMAX_DELAY);

        state_handle_message(msg);
    } // end for
}
