/**
 * @author Lundeen Cahilly
 * @date 2025-10-18
 *
 * Magnetometer task with non-blocking state machine for handling
 * magnetorquer interference. Task dispatches at 50 Hz to manage timing,
 * but magnetometer data updates at 10 Hz (20ms settle time gives 80%
 * magnetorquer duty cycle).
 */
#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

void magnetometer_task_init(slate_t *slate);
void magnetometer_task_dispatch(slate_t *slate);

extern sched_task_t magnetometer_task;
