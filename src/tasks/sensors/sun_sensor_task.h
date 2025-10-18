/**
 * @author Lundeen Cahilly
 * @date 2025-10-18
 *
 * Sun sensor task for reading photodiode ADCs and computing sun vector.
 * Runs at 20 Hz for fast attitude observations.
 */
#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

void sun_sensor_task_init(slate_t *slate);
void sun_sensor_task_dispatch(slate_t *slate);

extern sched_task_t sun_sensor_task;
