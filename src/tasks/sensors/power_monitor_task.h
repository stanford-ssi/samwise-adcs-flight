/**
 * @author Lundeen Cahilly
 * @date 2025-10-18
 *
 * Power monitor task for reading ADCS board power consumption.
 * Runs at 10 Hz (can be merged with magnetometer task if needed).
 */
#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

void power_monitor_task_init(slate_t *slate);
void power_monitor_task_dispatch(slate_t *slate);

extern sched_task_t power_monitor_task;
