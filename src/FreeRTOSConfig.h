#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define INCLUDE_xTimerPendFunctionCall  1

#define configSUPPORT_STATIC_ALLOCATION     1
#define configSUPPORT_DYNAMIC_ALLOCATION    0

#define configUSE_TIMERS                1
#define configTIMER_TASK_PRIORITY       ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH        10
#define configTIMER_TASK_STACK_DEPTH    256

/* --- RP2350 ARM required settings (do not change) --- */
#define configENABLE_MPU                    0
#define configENABLE_TRUSTZONE              0
#define configRUN_FREERTOS_SECURE_ONLY      1

/* --- Core count: 1 = single-core, 2 = SMP --- */
#define configNUMBER_OF_CORES               1
#define configTICK_CORE                     0

/* --- Basic FreeRTOS config --- */
#define configUSE_TASK_NOTIFICATIONS        1
#define configUSE_PREEMPTION                1
#define configUSE_IDLE_HOOK                 0
#define configUSE_TICK_HOOK                 0
#define configCPU_CLOCK_HZ                  ( SystemCoreClock )
#define configTICK_RATE_HZ                  ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                5
#define configMINIMAL_STACK_SIZE            ( configSTACK_DEPTH_TYPE ) 256
#define configMAX_TASK_NAME_LEN             16
#define configUSE_16_BIT_TICKS              0
#define configIDLE_SHOULD_YIELD             1

/* --- Optional: FPU state save (set 1 if using floats in tasks) --- */
#define configENABLE_FPU                    1

/* --- Sync primitives --- */
#define configUSE_MUTEXES                   1
#define configUSE_RECURSIVE_MUTEXES         1
#define configUSE_SEMAPHORES                1
#define configUSE_COUNTING_SEMAPHORES       1
#define configQUEUE_REGISTRY_SIZE           8
#define configUSE_QUEUE_SETS                0

/* --- Stack overflow detection (2 = most thorough) --- */
#define configCHECK_FOR_STACK_OVERFLOW      2

/* --- Required hook stubs (implement or leave as-is) --- */
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_xTaskGetSchedulerState      1
#define INCLUDE_xTaskDelayUntil             1

/* Interrupt priority config for CM33 */
#define configPRIO_BITS                     4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5
#define configKERNEL_INTERRUPT_PRIORITY     ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

#define configASSERT(x) do { if (!(x)) for(;;); } while(0)

#endif /* FREERTOS_CONFIG_H */
