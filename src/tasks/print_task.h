/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * Simple test task to print stuff.
 *
 */
#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

void print_task_init(slate_t *slate);

void print_task_dispatch(slate_t *slate);

extern sched_task_t print_task;