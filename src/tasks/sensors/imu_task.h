/**
 * @author Lundeen Cahilly, Niklas Vainio
 * @date 2025-10-18
 *
 * Fast IMU task for reading gyroscope data at 50 Hz.
 * This is the primary sensor for the attitude filter and control loops.
 */
#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

void imu_task_init(slate_t *slate);
void imu_task_dispatch(slate_t *slate);

extern sched_task_t imu_task;
