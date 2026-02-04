/*
 * @author Lundeen Cahilly
 * @date 2026-02-04
 *
 * This file implements all orbit propagators for testing purposes. It includes:
 * (i) Polar orbit propagator (fixed COEs)
 * (ii) TODO: Keplerian orbit propagator
 * (iii) TODO: rk4, perturbed dynamics, etc.
 */

#include "orbit_prop.h"

/* This function propagates the orbit of a satellite in a circular polar orbit
 * with the following orbital elements (COEs): a = 500km + R_E; e = 0; i = 90
 * deg; W = 0; w = 0; nu = 0;
 *
 * @param t time since t0 [s] (t0 = 0)
 * @return state vector [r_eci, v_eci] in km and km/s respectively
 */
void propagate_polar_orbit(float3 &r_eci, float3 &v_eci, float t)
{
    // Define semi-major axis [km]
    float a = 500.0f + R_E;

    // Calculate mean motion [rad/s]
    float n = sqrtf(MU_EARTH / (a * a * a));

    // Calculate position, velocity in ECI frame
    r_eci = {a * cosf(n * t), 0.0f, a * sinf(n * t)};          // [km]
    v_eci = {-a * n * sinf(n * t), 0.0f, a * n * cosf(n * t)}; // [km/s]
}

#ifdef TEST
void propagate_polar_orbit_test()
{
    LOG_INFO("=>=>=> Testing polar orbit propagation... <=<=<=");

    // Time parameters
    float t0 = 0.0f;
    float dt = 60.0f;        // [s] 60s timesteps
    float a = 500.0f + R_E; // [km] semi-major axis
    float T = 2 * 3.14159f * sqrtf(a * a * a / MU_EARTH); // [s] orbital period
    float tf = 90 * 60; // [s] 90 minutes ~1 orbit
    int steps = static_cast<int>((tf - t0) / dt);
    
    // Initialize state vectors
    float3 r_eci;
    float3 v_eci;

    // Propagate orbit
    for (int i = 0; i < steps; i++)
    {
        float t = t0 + i * dt;
        propagate_polar_orbit(r_eci, v_eci, t);
        LOG_INFO("time = %.1f s, r_eci = [%.3f, %.3f, %.3f] km, v_eci = [%.3f, %.3f, %.3f] km/s",
                 t, r_eci.x, r_eci.y, r_eci.z, v_eci.x, v_eci.y, v_eci.z);
    }

    bool passed = true; // TODO: Add actual test
    LOG_INFO("=>=>=> Test %s <=<=<=\n", passed ? "PASSED" : "FAILED");
}
#endif