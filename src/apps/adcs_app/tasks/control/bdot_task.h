/**
 * @author Niklas Vainio
 * @date 2025-06-02
 *
 * This task is responsible for running the B-dot detumbling algorithm
 *
 */
#pragma once

#include "apps/adcs_app/scheduler/state_machine_types.h"
#include "apps/adcs_app/slate.h"

void bdot_task_init(slate_t *slate);

void bdot_task_dispatch(slate_t *slate);

extern sched_task_t bdot_task;
