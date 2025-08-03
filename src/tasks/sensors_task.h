/**
 * @author Niklas Vainio
 * @date 2025-06-02
 *
 * This task is responsible for reasding data from sensors (magmeter, IMU, GPS,
 * sun sensors) and putting in in the slate. It runs in all states.
 *
 */
#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

void sensors_task_init(slate_t *slate);

void sensors_task_dispatch(slate_t *slate);

extern sched_task_t sensors_task;