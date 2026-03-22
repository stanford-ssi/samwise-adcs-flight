/**
 * @author Lundeen Cahilly
 * @date 2025-10-18
 */

#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

void gps_task_init(slate_t *slate);
void gps_task_dispatch(slate_t *slate);

extern sched_task_t gps_task;
