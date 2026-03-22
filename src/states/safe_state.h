/**
 * @author Matthew Musson, Lundeen Cahilly
 * @date 2025-11-08
 *
 * This file defines our default safe state that reads sensors and does nothing
 * else
 */

#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

sched_state_t *safe_get_next_state(slate_t *slate);

extern sched_state_t safe_state;