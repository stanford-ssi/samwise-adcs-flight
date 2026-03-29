#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

typedef enum {
    STATE_SAFE = 0,
    STATE_ENABLED,
    NUM_STATES
} StateId_t;

typedef enum {
    MSG_OFF = 0,
    MSG_ON,
    MSG_IDLE
} StateMsg_t;

typedef struct {
    EventBits_t enabled_bits;
} StateConfig_t;

#define SENSOR_QUEUE_DEPTH 4

typedef struct {
    StateConfig_t state_configs[NUM_STATES];

    StaticQueue_t state_queue;
    uint8_t state_queue_buffer[SENSOR_QUEUE_DEPTH * sizeof(StateMsg_t)];

    EventGroupHandle_t events;
    StaticEventGroup_t event_group;

} StateMachine_t;

void init_state_machine();
