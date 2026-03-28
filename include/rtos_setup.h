#pragma once
#include "FreeRTOS.h"

// Required by FreeRTOS when configSUPPORT_STATIC_ALLOCATION = 1
extern "C" void vApplicationGetIdleTaskMemory(
    StaticTask_t **ppxIdleTaskTCBBuffer,
    StackType_t **ppxIdleTaskStackBuffer,
    configSTACK_DEPTH_TYPE *puxIdleTaskStackSize)
{
    static StaticTask_t idle_tcb;
    static StackType_t  idle_stack[configMINIMAL_STACK_SIZE];
    *ppxIdleTaskTCBBuffer   = &idle_tcb;
    *ppxIdleTaskStackBuffer = idle_stack;
    *puxIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

// Required because configUSE_TIMERS = 1
extern "C" void vApplicationGetTimerTaskMemory(
    StaticTask_t **ppxTimerTaskTCBBuffer,
    StackType_t **ppxTimerTaskStackBuffer,
    configSTACK_DEPTH_TYPE *puxTimerTaskStackSize)
{
    static StaticTask_t timer_tcb;
    static StackType_t  timer_stack[configTIMER_TASK_STACK_DEPTH];
    *ppxTimerTaskTCBBuffer   = &timer_tcb;
    *ppxTimerTaskStackBuffer = timer_stack;
    *puxTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}

// FreeRTOS hook — required when configCHECK_FOR_STACK_OVERFLOW > 0
extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char *pcTaskName) {
    panic("Stack overflow in task: %s", pcTaskName);
}
