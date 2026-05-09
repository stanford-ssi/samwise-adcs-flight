#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "apps/adcs_new/states/states.h"
#include "apps/adcs_new/tasks/tasks.h"
#include "apps/adcs_new/slate.h"

extern slate_t slate;

void init_state_machine() {
    slate.state_machine.events = 
        xEventGroupCreateStatic(&slate.state_machine.event_group);

    slate.state_machine.state_queue_handle = xQueueCreateStatic(SENSOR_QUEUE_DEPTH,
            sizeof(StateMsg_t),
            slate.state_machine.state_queue_buffer,
            &slate.state_machine.state_queue);
    
    slate.state_machine.state_configs[STATE_INIT].enabled_bits
        = TASK_BIT(TASK_INIT);

    slate.state_machine.state_configs[STATE_ENABLED].enabled_bits
        = TASK_BIT(TASK_WATCHDOG)
        | TASK_BIT(TASK_BDOT)
        | TASK_BIT(TASK_ACTUATORS)
        | TASK_BIT(TASK_TELEMETRY)
        | TASK_BIT(TASK_IMU)
        | TASK_BIT(TASK_SUN_SENSOR)
        | TASK_BIT(TASK_GPS)
        | TASK_BIT(TASK_MAGNETOMETER);

    slate.state_machine.state_configs[STATE_SAFE].enabled_bits
        = TASK_BIT(TASK_WATCHDOG) 
        | TASK_BIT(TASK_TELEMETRY)
        | TASK_BIT(TASK_IMU)
        | TASK_BIT(TASK_GPS)
        | TASK_BIT(TASK_MAGNETOMETER);

    slate.state_machine.state_configs[STATE_DISABLED].enabled_bits
        = TASK_BIT(TASK_WATCHDOG)
        | TASK_BIT(TASK_TELEMETRY);

    slate.state_machine.state_configs[STATE_FUSION].enabled_bits
        =  TASK_BIT(TASK_WATCHDOG) 
        | TASK_BIT(TASK_TELEMETRY)
        | TASK_BIT(TASK_IMU) | TASK_BIT(TASK_GPS)
        | TASK_BIT(TASK_MAGNETOMETER)
        | TASK_BIT(TASK_PROPAGATE) | TASK_BIT(TASK_RESET_ESTIMATE)
        | TASK_BIT(TASK_MAGNETOMETER_FUSION)
        | TASK_BIT(TASK_SUN_VECTOR_FUSION)
        ;
}

void enter_state(StateId_t new_state) {
    switch (new_state) {
        case STATE_SAFE:
            break;
        case STATE_ENABLED:
            break;
        case STATE_DISABLED:
            break;
        case STATE_FUSION:
            for (int i = 0; i < 5; i++) {
                LOG_INFO("////////////////////////");
                LOG_INFO("SWITCHING TO FUSION MODE");
            }
            LOG_INFO("////////////////////////");
            break;
    };

    EventBits_t wanted = slate.state_machine.state_configs[new_state].enabled_bits;
    EventBits_t all = (1 << NTASKS) - 1; // NTASKS 1s

    // Disable tasks not within state
    xEventGroupClearBits(slate.state_machine.events, all & ~wanted);
    // Enable tasks within the state
    xEventGroupSetBits(slate.state_machine.events, wanted);

    // Update state!
    slate.state_machine.current_state = new_state;
}

void state_handle_message(StateMsg_t msg)
{
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
}

