#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

typedef enum {
    STATE_SAFE = 0,
    STATE_INIT,
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_FUSION,
    // Appended to preserve the existing 0-4 telemetry values.
    STATE_DETUMBLE,
    NUM_STATES
} StateId_t;

typedef enum {
    MSG_OFF = 0,
    MSG_INIT_DONE,
    MSG_ON,
    MSG_VOLTAGE_LOW,
    MSG_VOLTAGE_OK, // Bus recovered; the only way out of STATE_SAFE
    MSG_GPS_VALID,
    MSG_COMMAND_DISABLE,
    MSG_COMMAND_ENABLE_NORMAL,
    MSG_COMMAND_ENTER_DETUMBLE
} StateMsg_t;

typedef struct {
    EventBits_t enabled_bits;
} StateConfig_t;

#define SENSOR_QUEUE_DEPTH 4

typedef struct {
    // Keep track of the current state!
    StateId_t current_state;
    StateConfig_t state_configs[NUM_STATES];

    StaticQueue_t state_queue;
    uint8_t state_queue_buffer[SENSOR_QUEUE_DEPTH * sizeof(StateMsg_t)];

    EventGroupHandle_t events;
    StaticEventGroup_t event_group;

    TaskHandle_t state_machine_handler;

    QueueHandle_t state_queue_handle;

} StateMachine_t;

void init_state_machine();

void enter_state(StateId_t state);

void state_handle_message(StateMsg_t msg);
