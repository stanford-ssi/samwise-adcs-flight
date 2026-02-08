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
    bool passed = true; // prove me wrong

    // Time parameters
    float t0 = 0.0f;
    float dt = 1.0f;        // [s] 1.0s timestep
    float a = 500.0f + R_E; // [km] semi-major axis
    float T = 2 * 3.14159f * sqrtf(a * a * a / MU_EARTH); // [s] orbital period
    int steps = static_cast<int>((T - t0) / dt);          // one orbit

    // Initialize state vectors
    float3 r_eci;
    float3 v_eci;

    // header: time, r_eci, v_eci
    // make the header work with the data of the table
    printf("| %6s | %10s | %10s | %10s | %10s | %10s | %10s |\n", "time", "x",
           "y", "z", "vx", "vy", "vz");

    for (int i = 0; i < steps; i++)
    {
        float t = t0 + i * dt;
        propagate_polar_orbit(r_eci, v_eci, t);

        if (i % 60 == 0 || i == steps - 1)
        { // log every 60 steps and the last step
            // print with equal spacing like a table
            // header: time, r_eci, v_eci
            printf("| %6.1f | %10.3f | %10.3f | %10.3f | %10.6f | %10.6f | "
                   "%10.6f |\n",
                   t, r_eci.x, r_eci.y, r_eci.z, v_eci.x, v_eci.y, v_eci.z);
        }

        // Assert that the r is constant
        if (fabsf(length(r_eci) - a) > 1e-3f)
        {
            LOG_ERROR("r_eci is not constant: %.3f km", length(r_eci));
            passed = false;
            break;
        }

        // Assert that the v is constant
        if (fabsf(length(v_eci) - sqrtf(MU_EARTH / a)) > 1e-3f)
        {
            LOG_ERROR("v_eci is not constant: %.3f km/s", length(v_eci));
            passed = false;
            break;
        }
    }

    LOG_INFO("=>=>=> Test %s <=<=<=\n", passed ? "PASSED" : "FAILED");
}
#endif