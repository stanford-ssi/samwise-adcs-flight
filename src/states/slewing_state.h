/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines a state for attitude slewing.
 */
#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

sched_state_t *slewing_get_next_state(slate_t *slate);

extern sched_state_t slewing_state;