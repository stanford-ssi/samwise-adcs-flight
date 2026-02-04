/*
 * @author Lundeen Cahilly
 * @date 2026-02-04
 *
 * This file implements an attitude propagator for testing purposes.
 */

#include "attitude_propagator.h"
#include "constants.h"

/* This function propagates the attitude of the satellite given the current
 * ground truth state vector, actuator control inputs, and a time step dt. The
 * attitude is propagated using an RK4 integration scheme.
 */
void propagate_attitude(float6 x, float3 tau_gravity_gradient, float3 tau_drag, float3 tau_reaction_wheel, float3 tau_magnetorquer, float dt)
{
    // TODO: Implement attitude propagation
}

/* 
 * This function computes the gravity gradient torque on the satellite using two steps:
 * 
 * tau(r_eci, q_eci2body) = -3μ/r^5 * r_eci x (I_body r_body)
 */
static float3 tau_gravity_gradient(float4 q_eci2body, float3 r_eci) {
    // Convert r_eci to body frame
    float3 r_body = qrot(q_eci2body, r_eci);

    // Compute constant term
    float r_mag = length(r_eci);
    float r5 = r_mag * r_mag * r_mag * r_mag * r_mag;
    float constant = 3 * MU_EARTH / r5;

    // Compute the gravity gradient torque
    float3 I_r_body = mul(I_BODY, r_body);
    float3 tau_gg = constant * cross(r_body, I_r_body);
    return tau_gg;
}


#ifdef TEST

/* This function tests the gravity gradient torque computation. 
 */
void tau_gravity_gradient_test() {
    LOG_INFO("=>=>=> Testing gravity gradient torque... <=<=<=");
    float4 q_eci2body = {0.70710678f, 0, 0, 0.70710678f}; // 90 deg about X
    float3 r_eci = {6378.137f, 0.0f, 0.0f}; // Equator at prime meridian
    float3 tau_gg = tau_gravity_gradient(q_eci2body, r_eci);
    LOG_INFO("tau_gg: %f %f %f", tau_gg[0], tau_gg[1], tau_gg[2]);
    bool passed = true; // TODO: Add actual test
    printf("\n");
    LOG_INFO("=>=>=> Test %s <=<=<=", passed ? "PASSED" : "FAILED");
}

#endif