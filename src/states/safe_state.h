/**
 * @file safe_state.h
 *
 */
#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

sched_state_t *safe_get_next_state(slate_t *slate);

extern sched_state_t safe_state;