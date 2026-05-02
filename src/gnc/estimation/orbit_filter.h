/**
 * @author Lundeen Cahilly
 * @date 2025-10-25
 *
 * This file implements an orbit Kalman filter using RK4 integration
 * for position and velocity estimation from GPS measurements
 */

#pragma once

#include "linalg.h"
#include "slate.h"

using namespace linalg::aliases;

void orbit_filter_init(slate_t *slate);

void orbit_filter_propagate(slate_t *slate);

void orbit_filter_update(slate_t *slate);

#ifdef TEST
void orbit_filter_stationary_test(slate_t *slate);
void orbit_filter_convergence_test(slate_t *slate);
void orbit_filter_multi_orbit_test(slate_t *slate);
void orbit_filter_multi_orbit_gps_test(slate_t *slate);
#endif
