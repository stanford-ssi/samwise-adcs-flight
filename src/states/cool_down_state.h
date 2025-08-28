/**
 * @author Niklas Vainio
 * @date 2025-05-29
 *
 * This file defines the emergency cool down state: it reads the sensors but
 * does nothing.
 */
#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

sched_state_t *cool_down_get_next_state(slate_t *slate);

extern sched_state_t cool_down_state;