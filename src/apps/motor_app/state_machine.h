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
    TASK_STATE_MACHINE = 0,
    TASK_WATCHDOG,
    TASK_CONTROL,
    TASK_TELEMETRY,
    TASK_POWER,
    NTASKS
} TaskId_t;

typedef enum {
    MSG_OFF = 0,
    MSG_ON,
    MSG_IDLE
} StateMsg_t;

#define TASK_BIT(id) (1 << (id))

typedef struct {
    EventBits_t enabled_bits;
} StateConfig_t;
void enter_state(StateId_t new_state);

void init_state_machine(void);

void state_machine_task(void *);

