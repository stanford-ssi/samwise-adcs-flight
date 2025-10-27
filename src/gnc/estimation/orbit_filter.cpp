/**
 * @author Lundeen Cahilly
 * @date 2025-10-25
 *
 * This file implements an orbit Kalman filter using RK4 propagation
 * for position and velocity estimation from GPS measurements
 */

#include "orbit_filter.h"
#include "constants.h"
#include "gnc/utils/matrix_utils.h"
#include "gnc/utils/transforms.h"
#include "linalg.h"
#include "macros.h"
#include "pico/time.h"

// ========================================================================
//      COVARIANCE MATRICES
// ========================================================================

// Process noise covariance matrix - TODO: tune these values
// Models uncertainty in 2-body dynamics (unmodeled perturbations like J2, drag)
constexpr float POSITION_PROCESS_VARIANCE = 1e-6f; // m^2
constexpr float VELOCITY_PROCESS_VARIANCE = 1e-8f; // (m/s)^2
static float Q[6 * 6] = {
    POSITION_PROCESS_VARIANCE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    POSITION_PROCESS_VARIANCE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    POSITION_PROCESS_VARIANCE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    VELOCITY_PROCESS_VARIANCE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    VELOCITY_PROCESS_VARIANCE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    VELOCITY_PROCESS_VARIANCE,
};

// Measurement noise covariance matrix - TODO: tune from GPS datasheet
// GPS position accuracy (typical consumer GPS ~5-10m accuracy)
// GPS velocity accuracy from Doppler (typical ~0.1 m/s)
constexpr float GPS_POSITION_VARIANCE = 10.0f * 10.0f; // 10m standard deviation
constexpr float GPS_VELOCITY_VARIANCE =
    0.1f * 0.1f; // 0.1 m/s standard deviation

static float R[6 * 6] = {
    GPS_POSITION_VARIANCE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    GPS_POSITION_VARIANCE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    GPS_POSITION_VARIANCE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    GPS_VELOCITY_VARIANCE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    GPS_VELOCITY_VARIANCE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    GPS_VELOCITY_VARIANCE,
};

// Measurement matrix H (identity - we measure full state [r; v])
// h(x) = [I_6x6] * [r; v] = [r; v]
static float H[6 * 6] = {
    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
};

// ========================================================================
//      DYNAMICS UTILITIES
// ========================================================================

/**
 * @brief Compute the state derivative x_dot = f(x)
 * where x = [r; v] (position and velocity in ECI)
 * Uses simple 2-body dynamics: a = -μ/r³ * r
 *
 * @param x_dot Output state derivative (6-vector)
 * @param x Current state (6-vector)
 */
void compute_orbit_x_dot(float *x_dot, const float *x)
{
    // Extract position and velocity
    float3 r = {x[0], x[1], x[2]}; // km
    float3 v = {x[3], x[4], x[5]}; // km/s

    // Compute orbital dynamics: a = -μ/r³ * r
    float r_mag = length(r);
    float r_cubed = r_mag * r_mag * r_mag;
    float3 a = -(MU_EARTH / r_cubed) * r; // km/s^2

    // State derivative: [r_dot; v_dot] = [v; a]
    x_dot[0] = v.x;
    x_dot[1] = v.y;
    x_dot[2] = v.z;
    x_dot[3] = a.x;
    x_dot[4] = a.y;
    x_dot[5] = a.z;
}

/**
 * @brief RK4 integration step for state propagation
 *
 * @param x_new Output propagated state (6-vector)
 * @param x Current state (6-vector)
 * @param dt Time step [seconds]
 */
void rk4_step(float *x_new, const float *x, float dt)
{
    float k1[6], k2[6], k3[6], k4[6];
    float x_temp[6];

    // k1 = f(x)
    compute_orbit_x_dot(k1, x);

    // k2 = f(x + dt/2 * k1)
    for (int i = 0; i < 6; i++)
    {
        x_temp[i] = x[i] + 0.5f * dt * k1[i];
    }
    compute_orbit_x_dot(k2, x_temp);

    // k3 = f(x + dt/2 * k2)
    for (int i = 0; i < 6; i++)
    {
        x_temp[i] = x[i] + 0.5f * dt * k2[i];
    }
    compute_orbit_x_dot(k3, x_temp);

    // k4 = f(x + dt * k3)
    for (int i = 0; i < 6; i++)
    {
        x_temp[i] = x[i] + dt * k3[i];
    }
    compute_orbit_x_dot(k4, x_temp);

    // x_new = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    for (int i = 0; i < 6; i++)
    {
        x_new[i] =
            x[i] + (dt / 6.0f) * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]);
    }
}

// ========================================================================
//      ORBIT FILTER STEPS
// ========================================================================

/**
 * @brief Initialize the orbit filter state and covariance
 *
 * @param slate Pointer to the ADCS slate structure
 */
void orbit_filter_init(slate_t *slate)
{
    // TODO: Initialize position and velocity from GPS
    // TODO: Set initial covariance matrix P
    // TODO: Set filter initialization flag

    LOG_INFO("Initialized orbit filter!");
}

/**
 * @brief Propagate the orbit filter state and covariance using RK4 integration
 * over the time step since last propagate.
 *
 * @param slate Pointer to the ADCS slate structure
 */
void orbit_filter_propagate(slate_t *slate)
{
    // TODO: Get dt since last propagate
    // TODO: RK4 propagate state: x_new = rk4_step(x, dt)
    // TODO: Propagate covariance using state transition matrix Phi
    //       (linearized dynamics for covariance only)
    // TODO: Write propagated state back to slate (r_eci, v_eci)

    LOG_DEBUG("[orbit_filter] Propagated orbit state");
}

/**
 * @brief Update the orbit filter state and covariance using GPS position
 * measurement.
 *
 * @param slate Pointer to the ADCS slate structure
 */
void orbit_filter_update(slate_t *slate)
{
    // TODO: Convert GPS lat/lon/alt to r_eci (ECEF -> ECI transformation)
    // TODO: Convert GPS speed/course to v_eci (ECEF -> ECI transformation)
    // TODO: Compute Kalman gain: K = P @ H^T @ (H @ P @ H^T + R)^-1
    //       where H = I_6x6 (identity - we measure full state)
    // TODO: Compute innovation: y = [r_measured; v_measured] - [r_estimated;
    // v_estimated]
    // TODO: Update state: x_new = x + K @ y
    // TODO: Update covariance: P_new = (I - K @ H) @ P
    // TODO: Write updated state back to slate

    LOG_DEBUG("[orbit_filter] Updated orbit state from GPS");
}

#ifdef TEST
void orbit_filter_convergence_test(slate_t *slate)
{
    // TODO: Test filter convergence with simulated GPS measurements
}

void orbit_filter_rk4_accuracy_test(slate_t *slate)
{
    // TODO: Compare RK4 propagation accuracy vs analytical orbit
}
#endif
