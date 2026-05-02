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
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_INIT));
        LOG_ERROR("Reentered init function?");
    }
}

void vTaskWatchdog(void *) {
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_WATCHDOG));
        TASK_LOOP_MS(100);
        watchdog_feed(&slate.watchdog);

        // ALSO HANDLE POWER MONITOR
        bool result = adm_get_power(&slate.power_monitor);
        LOG_INFO("[sensor] P = %.3fW, V = %.3fV, I = %.3fA", 
                slate.power_monitor.power,
                slate.power_monitor.voltage, 
                slate.power_monitor.current);

        if (slate.power_monitor.voltage < ADCS_VOLTAGE_UNSAFE) {
            StateMsg_t msg = MSG_VOLTAGE_LOW;
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

        switch(slate.state_machine.current_state) {
            case STATE_DISABLED:
                switch(msg) {
                    case MSG_COMMAND_RECEIVED:
                        enter_state(STATE_ENABLED);
                        break;
                }
            case STATE_INIT:
                switch(msg) {
                    case MSG_INIT_DONE:
                        enter_state(STATE_ENABLED);
                        break;
                }
                break;
            case STATE_ENABLED:
                switch(msg) {
                    case MSG_VOLTAGE_LOW:
                        enter_state(STATE_SAFE);
                        break;
                    case MSG_GPS_VALID:
                        enter_state(STATE_FUSION);
                        break;
                    case MSG_COMMAND_RECEIVED:
                        enter_state(STATE_DISABLED);
                        break;
                }
                break;
            case STATE_FUSION:
                switch(msg) {
                    case MSG_VOLTAGE_LOW:
                        enter_state(STATE_SAFE);
                        break;
                    case MSG_COMMAND_RECEIVED:
                        enter_state(STATE_DISABLED);
                        break;
                }
                break;
        } // end switch (state)
    } // end for
}
