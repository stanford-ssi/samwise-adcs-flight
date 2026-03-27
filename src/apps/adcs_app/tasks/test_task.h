/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * Task to run tests - feel free to hook new code in here
 *
 */
#pragma once

#include "apps/adcs_app/scheduler/state_machine_types.h"
#include "apps/adcs_app/slate.h"

void test_task_init(slate_t *slate);
void test_task_dispatch(slate_t *slate);

extern sched_task_t test_task;
