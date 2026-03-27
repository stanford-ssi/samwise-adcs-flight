/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This task is responsible for sending telemetry to the picubed. It runs in all
 * states.
 */
#pragma once

#include "apps/adcs_app/scheduler/state_machine_types.h"
#include "apps/adcs_app/slate.h"

void telemetry_task_init(slate_t *slate);

void telemetry_task_dispatch(slate_t *slate);

extern sched_task_t telemetry_task;
