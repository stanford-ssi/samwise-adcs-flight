/**
 * @author Lundeen Cahilly
 * @date 2025-08-13
 *
 * This task is responsible for feeding the watchdog timer to prevent system
 * resets. Should run in all states.
 */

#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

void watchdog_task_init(slate_t *slate);

void watchdog_task_dispatch(slate_t *slate);

extern sched_task_t watchdog_task;