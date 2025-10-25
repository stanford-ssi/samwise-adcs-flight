/**
 * @author Lundeen Cahilly
 * @date 2025-10-24
 *
 * This task is responsible for sending telemetry to the motor board to run
 * the reaction wheels
 */
#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

void reaction_wheel_task_init(slate_t *slate);

void reaction_wheel_task_dispatch(slate_t *slate);

extern sched_task_t reaction_wheel_task;