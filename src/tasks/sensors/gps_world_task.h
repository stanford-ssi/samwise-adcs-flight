/**
 * @author Lundeen Cahilly
 * @date 2025-10-18
 *
 * GPS and world model task. Reads GPS at 1 Hz and updates reference vectors
 * (b_eci, sun_vector_eci) and position (r_eci, r_ecef) when GPS data is valid.
 */
#pragma once

#include "scheduler/state_machine_types.h"
#include "slate.h"

void gps_world_task_init(slate_t *slate);
void gps_world_task_dispatch(slate_t *slate);

extern sched_task_t gps_world_task;
