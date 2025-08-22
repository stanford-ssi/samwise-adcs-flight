/**
 * @author Lundeen Cahilly
 * @date 2025-08-14
 *
 * This task is responsible for driving the magnetorquers and reaction wheels
 * at the desired rates as indicated on the slate. It runs in all states where
 * actuators are needed.
 */
#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

void actuators_task_init(slate_t *slate);

void actuators_task_dispatch(slate_t *slate);

extern sched_task_t actuators_task;