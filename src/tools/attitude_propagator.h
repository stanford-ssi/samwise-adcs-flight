/*
 * @author Lundeen Cahilly
 * @date 2026-02-04
 *
 * This file implements an attitude propagator for testing purposes. It is used
 * to propagate the attitude of the satellite given the current ground truth
 * state vector, actuator control inputs, and a time step dt. The attitude is
 * propagated using an RK4 integration scheme.
 */

#include "constants.h"
#include "linalg.h"
#include "macros.h"

using namespace linalg::aliases;

void propagate_attitude(float4& q_eci2body, float3& w_eci, float3 r_eci, float3 v_eci,
                        float3 mu_mag, float3 tau_rw, float dt);

#ifdef TEST
void tau_gravity_gradient_test();
void tau_drag_test();
#endif