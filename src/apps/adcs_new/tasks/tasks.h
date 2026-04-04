#pragma once

#include "macros.h"

typedef enum {
    TASK_STATE_MACHINE = 0,
    TASK_INIT,
    TASK_WATCHDOG,
    TASK_MAGNETOMETER_FUSION,
    TASK_SUN_VECTOR_FUSION,
    TASK_PROPAGATE,
    TASK_RESET_ESTIMATE,
    TASK_GPS,
    TASK_IMU,
    TASK_MAGNETOMETER,
    TASK_SUN_SENSOR,
    TASK_CALCULATE_TORQUE,
    TASK_REACTION_WHEEL,
    TASK_MAGNETORQUER,
    TASK_PICUBED_SEND,
    TASK_PICUBED_RECEIVE,
    NTASKS
} TaskId_t;

#define TASK_BIT(id) (1 << (id))

#define TASK_LOOP_MS(ms) TickType_t xLastWakeTime = xTaskGetTickCount(); \
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ms))

#define WAIT_UNTIL_EVENTBIT(bit) \
    xEventGroupWaitBits(slate.state_machine.events, \
        (bit),         \
        pdFALSE,                    \
        pdTRUE,                     \
        portMAX_DELAY)

// ==================================================================
// TASK RELATED UTILITIES
// ==================================================================
// void adcs_tasks_init(void) {
//     LOG_INFO("TODO: INIT");
// }

// ==================================================================
// SENSOR FUSION TASKS
// ==================================================================
void vTaskMagnetometerFusion(void *);

void vTaskSunVectorFusion(void *);

void vTaskPropagate(void *);

void vTaskResetEstimate(void *);

// ==================================================================
// SENSOR TASKS
// ==================================================================
void vTaskGPS(void *);

void vTaskIMU(void *);

void vTaskMagnetometer(void *);

void vTaskSunSensor(void *);

// ==================================================================
// CONTROL TASKS
// ==================================================================

void vTaskControlCalculations(void *);

void vTaskReactionWheels(void *);

void vTaskMagnetorquers(void *);

// ==================================================================
// TELEMETRY TASKS
// ==================================================================

void vTaskPiCubedSend(void *);

void vTaskPiCubedReceive(void *);

// ==================================================================
// OTHER TASKS
// ==================================================================

void vTaskWatchdog(void *);

void vTaskStateMachine(void *);

void vTaskInit(void *);
