/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines a test state with test tasks
 */
#pragma once

#include "apps/adcs_app/scheduler/state_machine_types.h"
#include "apps/adcs_app/slate.h"

sched_state_t *test_get_next_state(slate_t *slate);

extern sched_state_t test_state;
