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
float6 propagate_polar_orbit(float t)
{
    // Define semi-major axis [km]
    float a = 500.0f + R_E;

    // Calculate mean motion [rad/s]
    float n = sqrtf(MU_EARTH / (a * a * a));

    // Calculate position, velocity in ECI frame
    float3 r_eci = {a * cos(n * t), 0.0f, a * sin(n * t)};          // [km]
    float3 v_eci = {-a * n * sin(n * t), 0.0f, a * n * cos(n * t)}; // [km/s]

    return float6(r_eci, v_eci);
}

#ifdef TEST
void propagate_polar_orbit_test()
{
    LOG_INFO("=>=>=> Testing polar orbit propagation... <=<=<=");

    // Time parameters
    float t0 = 0.0f;
    float dt = 1.0f;        // [s] 1s timesteps
    float a = 500.0f + R_E; // [km] semi-major axis
    float T = 2 * 3.14159f * sqrtf(a * a * a / MU_EARTH); // [s] orbital period
    float tf = 90 * 60; // [s] 90 minutes ~1 orbit
    int steps = static_cast<int>((tf - t0) / dt);

    // Propagate orbit
    for (int i = 0; i < steps; i++)
    {
        float t = t0 + i * dt;
        float6 x = propagate_polar_orbit(t);
        LOG_INFO(
            "r_eci = [%.3f, %.3f, %.3f] km, v_eci = [%.3f, %.3f, %.3f] km/s",
            x[0], x[1], x[2], x[3], x[4], x[5]);
    }

    bool passed = true; // TODO: Add actual test
    LOG_INFO("=>=>=> Test %s <=<=<=\n", passed ? "PASSED" : "FAILED");
}
#endif