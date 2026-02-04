/*
 * @author Lundeen Cahilly
 * @date 2026-02-04
 *
 * This file implements all orbit propagators for testing purposes. It includes:
 * (i) Polar orbit propagator (fixed COEs)
 * (ii) TODO: Keplerian orbit propagator
 * (iii) TODO: rk4, perturbed dynamics, etc.
 */

#include "constants.h"
#include "linalg.h"

using namespace linalg::aliases;

float6 propagate_polar_orbit(float dt);

#ifdef TEST
void propagate_polar_orbit_test();
#endif