#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "apps/motor_app/state_machine.h"
#include "apps/motor_app/motor_slate.h"

extern motor_slate_t motor_slate;

static StaticEventGroup_t event_group;

static StateConfig_t state_configs[NUM_STATES];

#define SENSOR_QUEUE_DEPTH 4
static StaticQueue_t state_queue;
uint8_t state_queue_buffer[SENSOR_QUEUE_DEPTH * sizeof(StateMsg_t)];

void init_state_machine() {
    motor_slate.events = xEventGroupCreateStatic(&event_group);

    motor_slate.state_queue_handle = xQueueCreateStatic(SENSOR_QUEUE_DEPTH,
            sizeof(StateMsg_t),
            state_queue_buffer,
            &state_queue);

    state_configs[STATE_ENABLED].enabled_bits = TASK_BIT(TASK_WATCHDOG) 
        | TASK_BIT(TASK_POWER)
        | TASK_BIT(TASK_TELEMETRY)| TASK_BIT(TASK_CONTROL);

    state_configs[STATE_SAFE].enabled_bits = TASK_BIT(TASK_WATCHDOG) 
        | TASK_BIT(TASK_POWER) | TASK_BIT(TASK_TELEMETRY);
}

void enter_state(StateId_t new_state) {
    switch (new_state) {
        case STATE_SAFE:
            break;
        case STATE_ENABLED:
            break;
    };

    EventBits_t wanted = state_configs[new_state].enabled_bits;
    EventBits_t all = (1 << NTASKS) - 1; // NTASKS 1s

    xEventGroupClearBits(motor_slate.events, all & ~wanted);
    xEventGroupSetBits(motor_slate.events, wanted);

    motor_slate.current_state = new_state;
}

void state_machine_task(void *) {
    enter_state(STATE_SAFE);

    for(;;) {
        StateMsg_t msg;
        // xTaskNotifyWait(0, UINT32_MAX, &msg, portMAX_DELAY);
        xQueueReceive(motor_slate.state_queue_handle,
                &msg, portMAX_DELAY);

        switch (motor_slate.current_state) {
            case STATE_SAFE:
                enter_state(STATE_ENABLED);
                break;
            case STATE_ENABLED:
                enter_state(STATE_SAFE);
                break;
        };
    }
}
