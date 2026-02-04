/*
 * @author Lundeen Cahilly
 * @date 2026-02-04
 *
 * This file implements an attitude propagator for testing purposes. It is used to
 * propagate the attitude of the satellite given the current ground truth state vector,
 * actuator control inputs, and a time step dt. The attitude is propagated using an RK4
 * integration scheme.
 */

#include "constants.h"
#include "linalg.h"

using namespace linalg::aliases;

void propagate_attitude(float6 x, float3 tau_gravity_gradient, float3 tau_drag, float3 tau_reaction_wheel, float3 tau_magnetorquer, float dt);

#ifdef TEST
void tau_gravity_gradient_test();
#endif