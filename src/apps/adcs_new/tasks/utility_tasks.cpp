#include "FreeRTOS.h"
#include "task.h"

#include "macros.h"

#include "apps/adcs_new/params.h"
#include "apps/adcs_new/slate.h"
#include "apps/adcs_new/states/states.h"
#include "apps/adcs_new/tasks/tasks.h"

#include "drivers/watchdog_motor/watchdog.h"

extern slate_t slate;

void vTaskWatchdog(void *) {
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_WATCHDOG));
        TASK_LOOP_MS(100);
        watchdog_feed(&slate.watchdog);
    }
}

void vTaskStateMachine(void *) {
    enter_state(STATE_SAFE);
    for(;;) {
        StateMsg_t msg; // Message
        xQueueReceive(slate.state_machine.state_queue_handle,
                &msg, 
                portMAX_DELAY);

        switch (msg) {
            case MSG_GPS_VALID:
                enter_state(STATE_FUSION);
                break;
            case MSG_ON:
                switch (slate.state_machine.current_state) {
                    case STATE_SAFE:
                        enter_state(STATE_ENABLED);
                        break;
                    case STATE_ENABLED:
                        enter_state(STATE_SAFE);
                        break;
                };
                break;
        }
    }
}
