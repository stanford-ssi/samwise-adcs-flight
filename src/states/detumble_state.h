/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines a state for detumbling
 */
#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

sched_state_t *detumble_get_next_state(slate_t *slate);

extern sched_state_t detumble_state;