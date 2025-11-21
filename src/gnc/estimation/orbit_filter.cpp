/**
 * @author Lundeen Cahilly
 * @date 2025-10-25
 *
 * This file implements an orbit Kalman filter using RK4 integration
 * for position and velocity estimation from GPS measurements.
 *
 * NOTE: GPS speed/course from RMC sentence provides horizontal velocity only.
 * Vertical velocity component is estimated by the filter from dynamics.
 *
 * TODO: Tune process and measurement noise covariances based on GPS and
 * simulations
 * TODO: Integrate into the complete GNC system
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
// Models uncertainty in 2-body dynamics (unmodeled perturbations like J2, drag,
// SRP) Units: km^2 and (km/s)^2 to match orbit filter state units These values
// account for ~1m position and ~1mm/s velocity drift per second
constexpr float POSITION_PROCESS_VARIANCE =
    1e-6f; // km^2 (~1m std dev per second)
constexpr float VELOCITY_PROCESS_VARIANCE =
    1e-12f; // (km/s)^2 (~1mm/s std dev per second)
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
// Units: km^2 and (km/s)^2 to match orbit filter state units
constexpr float GPS_POSITION_VARIANCE =
    0.01f * 0.01f; // (0.01 km)^2 = (10m)^2 std dev
constexpr float GPS_VELOCITY_VARIANCE =
    0.0001f * 0.0001f; // (0.0001 km/s)^2 = (0.1 m/s)^2 std dev

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
void compute_orbit_x_dot(float *x_dot, const float *x, const float3 &a_imu_body,
                         const quaternion &q_eci_to_body)
{
    // Extract position and velocity
    float3 r = {x[0], x[1], x[2]}; // km
    float3 v = {x[3], x[4], x[5]}; // km/s

    // Compute gravitational acceleration: a_grav = -μ/r³ * r
    float r_mag = length(r);
    float r_cubed = r_mag * r_mag * r_mag;
    float3 a_grav_eci = -(MU_EARTH / r_cubed) * r; // km/s^2

    // IMU measures specific force (non-gravitational acceleration) in body
    // frame Convert to ECI frame
    float3 a_imu_eci = body_to_eci(a_imu_body, q_eci_to_body);

    // Total acceleration = gravity + non-gravitational forces
    float3 a_total = a_grav_eci + a_imu_eci;

    // State derivative: [r_dot; v_dot] = [v; a_total]
    x_dot[0] = v.x;
    x_dot[1] = v.y;
    x_dot[2] = v.z;
    x_dot[3] = a_total.x;
    x_dot[4] = a_total.y;
    x_dot[5] = a_total.z;
}

/**
 * @brief RK4 integration step for state propagation with IMU
 *
 * @param x_new Output propagated state (6-vector)
 * @param x Current state (6-vector)
 * @param dt Time step [seconds]
 * @param a_imu_body IMU-measured specific force in body frame [km/s^2]
 * @param q_eci_to_body Attitude quaternion (ECI to body frame)
 */
void rk4_step(float *x_new, const float *x, float dt, const float3 &a_imu_body,
              const quaternion &q_eci_to_body)
{
    float k1[6], k2[6], k3[6], k4[6];
    float x_temp[6];

    // k1 = f(x)
    compute_orbit_x_dot(k1, x, a_imu_body, q_eci_to_body);

    // k2 = f(x + dt/2 * k1)
    for (int i = 0; i < 6; i++)
    {
        x_temp[i] = x[i] + 0.5f * dt * k1[i];
    }
    compute_orbit_x_dot(k2, x_temp, a_imu_body, q_eci_to_body);

    // k3 = f(x + dt/2 * k2)
    for (int i = 0; i < 6; i++)
    {
        x_temp[i] = x[i] + 0.5f * dt * k2[i];
    }
    compute_orbit_x_dot(k3, x_temp, a_imu_body, q_eci_to_body);

    // k4 = f(x + dt * k3)
    for (int i = 0; i < 6; i++)
    {
        x_temp[i] = x[i] + dt * k3[i];
    }
    compute_orbit_x_dot(k4, x_temp, a_imu_body, q_eci_to_body);

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
 * Equations:
 *   State vector: x = [r; v] where r is position [km] and v is velocity [km/s]
 *   in ECI frame
 *
 *   Initial covariance: P = diag([σ_r², σ_r², σ_r², σ_v², σ_v², σ_v²])
 *   where σ_r ~ 0.1 km (GPS position uncertainty + ECEF conversion)
 *         σ_v ~ 0.01 km/s (initial velocity uncertainty from speed/course)
 *
 * @param slate Pointer to the ADCS slate structure
 */
void orbit_filter_init(slate_t *slate)
{
    // Initialize IMU acceleration to zero (will be set by IMU driver in flight)
    slate->a_body = {0.0f, 0.0f, 0.0f};

    // Initialize position from GPS (LLA -> ECEF -> ECI)
    float3 r_ecef = lla_to_ecef(slate->gps_lat, slate->gps_lon, slate->gps_alt);
    slate->r_ecef = r_ecef;
    slate->r_eci = ecef_to_eci(r_ecef, slate->MJD);

    // Initialize velocity from GPS speed/course (ENU -> ECEF -> ECI)
    float3 v_enu =
        speed_course_to_enu_velocity(slate->gps_speed, slate->gps_course);
    float3 lla = {slate->gps_lat, slate->gps_lon, slate->gps_alt};
    float3 v_ecef = enu_to_ecef(v_enu, lla);
    slate->v_eci = ecef_to_eci(v_ecef, slate->MJD);

    // Initialize covariance matrix P
    // Initial position uncertainty: ~0.1 km (100m, conservative GPS +
    // conversion) Initial velocity uncertainty: ~0.01 km/s (10 m/s,
    // conservative)
    constexpr float INIT_POSITION_VARIANCE = 0.1f * 0.1f;   // km^2
    constexpr float INIT_VELOCITY_VARIANCE = 0.01f * 0.01f; // (km/s)^2

    for (int i = 0; i < 6 * 6; i++)
    {
        if (i % 7 == 0 && i < 18)
        {
            slate->P_orbit[i] = INIT_POSITION_VARIANCE;
        }
        else if (i % 7 == 0 && i >= 18)
        {
            slate->P_orbit[i] = INIT_VELOCITY_VARIANCE;
        }
        else
        {
            slate->P_orbit[i] = 0.0f;
        }
    }

    slate->P_orbit_log_frobenius = mat_log_frobenius(slate->P_orbit, 6);
    slate->of_is_initialized = true;
    slate->of_init_count++;
    slate->of_last_propagate_time = get_absolute_time();

    LOG_INFO("Initialized orbit filter! (%d times so far)",
             slate->of_init_count);
    LOG_DEBUG("[orbit_filter] r_eci = [%.3f, %.3f, %.3f] km", slate->r_eci.x,
              slate->r_eci.y, slate->r_eci.z);
    LOG_DEBUG("[orbit_filter] v_eci = [%.6f, %.6f, %.6f] km/s", slate->v_eci.x,
              slate->v_eci.y, slate->v_eci.z);
}

/**
 * @brief Propagate the orbit filter state and covariance using RK4 integration
 *
 * Equations:
 *   State propagation (RK4):
 *     x_{k+1} = RK4(x_k, dt) where x = [r; v]
 *     Uses 4th-order Runge-Kutta for better energy conservation
 *
 *   Covariance propagation (continuous-time):
 *     Ṗ = F*P + P*F^T + Q
 *     P_{k+1} = P_k + Ṗ*dt
 *   where F = [ 0    I   ]
 *             [ ∂a/∂r  0 ]
 *   and ∂a/∂r = -μ/r³ * (I - 3*r*r^T/r²)
 *
 * @param slate Pointer to the ADCS slate structure
 */
void orbit_filter_propagate(slate_t *slate)
{
    // Get dt since last propagate
    absolute_time_t current_time = get_absolute_time();
    if (!slate->of_last_propagate_time)
    {
        slate->of_last_propagate_time = get_absolute_time();
        return;
    }
    float dt =
        absolute_time_diff_us(slate->of_last_propagate_time, current_time) *
        1e-6f;
    slate->of_last_propagate_time = current_time;

    // Propagate state using RK4 with IMU acceleration
    float x[6] = {slate->r_eci.x, slate->r_eci.y, slate->r_eci.z,
                  slate->v_eci.x, slate->v_eci.y, slate->v_eci.z};
    float x_new[6];
    rk4_step(x_new, x, dt, slate->a_body, slate->q_eci_to_body);

    // Write back to slate
    slate->r_eci = {x_new[0], x_new[1], x_new[2]};
    slate->v_eci = {x_new[3], x_new[4], x_new[5]};

    // Compute Jacobian ∂a/∂r for covariance propagation
    float r_mag = length(slate->r_eci);
    float r_squared = r_mag * r_mag;
    float r_cubed = r_mag * r_squared;
    float3x3 rrt = outerprod(slate->r_eci, slate->r_eci);
    float3x3 da_dr =
        -(MU_EARTH / r_cubed) * (identity3x3 - 3.0f * rrt / r_squared);

    // Build F matrix [6x6]: F = [ 0    I   ]
    //                            [ da_dr  0 ]
    float F[6 * 6] = {0};
    for (int i = 0; i < 3; i++)
    {
        F[i * 6 + (i + 3)] = 1.0f; // Upper right: I
        for (int j = 0; j < 3; j++)
        {
            F[(i + 3) * 6 + j] = da_dr[i][j]; // Lower left: da_dr
        }
    }

    // Propagate covariance: Ṗ = F*P + P*F^T + Q
    float F_T[6 * 6];
    float FP[6 * 6];
    float PF_T[6 * 6];
    mat_transpose(F, F_T, 6, 6);
    mat_mul_square(F, slate->P_orbit, FP, 6);
    mat_mul_square(slate->P_orbit, F_T, PF_T, 6);

    float P_dot[6 * 6];
    mat_add(FP, PF_T, P_dot, 6, 6);

    // P_{k+1} = P_k + (F*P + P*F^T + Q)*dt
    for (int i = 0; i < 36; i++)
    {
        slate->P_orbit[i] = slate->P_orbit[i] + (P_dot[i] + Q[i]) * dt;
    }

    slate->P_orbit_log_frobenius = mat_log_frobenius(slate->P_orbit, 6);

    // LOG_DEBUG("[orbit_filter] r_eci = [%.3f, %.3f, %.3f] km", slate->r_eci.x,
    //           slate->r_eci.y, slate->r_eci.z);
    // LOG_DEBUG("[orbit_filter] v_eci = [%.6f, %.6f, %.6f] km/s",
    // slate->v_eci.x,
    //           slate->v_eci.y, slate->v_eci.z);
}

/**
 * @brief Update the orbit filter state and covariance using GPS measurement
 *
 * Equations:
 *   Measurement model: z = H*x + v where H = I_{6x6}, v ~ N(0, R)
 *
 *   Kalman gain: K = P*H^T*(H*P*H^T + R)^{-1}
 *   Innovation: y = z_measured - H*x_predicted
 *   State update: x_new = x + K*y
 *   Covariance update: P_new = (I - K*H)*P
 *
 * @param slate Pointer to the ADCS slate structure
 */
void orbit_filter_update(slate_t *slate)
{
    // Convert GPS position to ECI
    float3 r_ecef = lla_to_ecef(slate->gps_lat, slate->gps_lon, slate->gps_alt);
    float3 r_meas = ecef_to_eci(r_ecef, slate->MJD);

    // Convert GPS velocity to ECI
    float3 v_enu =
        speed_course_to_enu_velocity(slate->gps_speed, slate->gps_course);
    float3 lla = {slate->gps_lat, slate->gps_lon, slate->gps_alt};
    float3 v_ecef = enu_to_ecef(v_enu, lla);
    float3 v_meas = ecef_to_eci(v_ecef, slate->MJD);

    // Compute Kalman gain: K = P*H^T*(H*P*H^T + R)^{-1}
    // Since H = I, this simplifies to: K = P*(P + R)^{-1}
    float P_plus_R[6 * 6];
    float P_plus_R_inv[6 * 6];
    float K[6 * 6];
    mat_add(slate->P_orbit, R, P_plus_R, 6, 6);
    mat_inverse(P_plus_R, P_plus_R_inv, 6);
    mat_mul_square(slate->P_orbit, P_plus_R_inv, K, 6);

    // Compute innovation: y = z_measured - x_predicted
    float innovation[6] = {
        r_meas.x - slate->r_eci.x, r_meas.y - slate->r_eci.y,
        r_meas.z - slate->r_eci.z, v_meas.x - slate->v_eci.x,
        v_meas.y - slate->v_eci.y, v_meas.z - slate->v_eci.z,
    };

    // Update state: x_new = x + K*y
    float K_innovation[6];
    mat_mul(K, innovation, K_innovation, 6, 6, 1);

    slate->r_eci.x += K_innovation[0];
    slate->r_eci.y += K_innovation[1];
    slate->r_eci.z += K_innovation[2];
    slate->v_eci.x += K_innovation[3];
    slate->v_eci.y += K_innovation[4];
    slate->v_eci.z += K_innovation[5];

    // Update covariance: P_new = (I - K*H)*P = (I - K)*P (since H = I)
    float I_minus_K[6 * 6];
    mat_sub(identity6x6, K, I_minus_K, 6, 6);
    float P_new[6 * 6];
    mat_mul_square(I_minus_K, slate->P_orbit, P_new, 6);

    for (int i = 0; i < 36; i++)
    {
        slate->P_orbit[i] = P_new[i];
    }

    slate->P_orbit_log_frobenius = mat_log_frobenius(slate->P_orbit, 6);

    // LOG_DEBUG("[orbit_filter] Updated from GPS: r_eci = [%.3f, %.3f, %.3f]
    // km",
    //           slate->r_eci.x, slate->r_eci.y, slate->r_eci.z);
    // LOG_DEBUG("[orbit_filter] v_eci = [%.6f, %.6f, %.6f] km/s",
    // slate->v_eci.x,
    //           slate->v_eci.y, slate->v_eci.z);
}

#ifdef TEST
void orbit_filter_stationary_test(slate_t *slate)
{
    printf("\n><=><=><=><=><= Testing Orbit Filter (Stationary Ground) "
           "><=><=><=><=><=\n");

    // Equator at prime meridian (stationary on ground)
    slate->gps_lat = 0.0f;    // degrees (equator)
    slate->gps_lon = 0.0f;    // degrees (prime meridian)
    slate->gps_alt = 0.0f;    // km (sea level)
    slate->gps_speed = 0.0f;  // stationary
    slate->gps_course = 0.0f; // doesn't matter
    slate->MJD = 60000.0f;

    orbit_filter_init(slate);

    LOG_INFO("Initialized at equator (stationary on ground, sea level)");
    LOG_INFO("Initial r_eci: [%.3f, %.3f, %.3f] km (mag=%.3f)", slate->r_eci.x,
             slate->r_eci.y, slate->r_eci.z, length(slate->r_eci));
    LOG_INFO("Initial v_eci: [%.6f, %.6f, %.6f] km/s (mag=%.6f)",
             slate->v_eci.x, slate->v_eci.y, slate->v_eci.z,
             length(slate->v_eci));

    float r_initial_mag = length(slate->r_eci);
    float v_initial_mag = length(slate->v_eci);

    // Propagate for 10 seconds - should stay near Earth's surface
    // v_eci magnitude should stay constant (~0.465 km/s at equator from
    // rotation)
    int steps = 10;
    float dt = 1.0f;

    LOG_INFO("Propagating for %d steps (dt=%.1f s)", steps, dt);
    LOG_INFO("Expected: r stays at ~%.3f km, v_mag stays at ~%.6f km/s", R_E,
             v_initial_mag);

    bool passed = true;
    for (int i = 0; i < steps; i++)
    {
        slate->of_last_propagate_time =
            get_absolute_time() - static_cast<uint64_t>(dt * 1e6);

        orbit_filter_propagate(slate);

        // Check for NaN
        if (std::isnan(slate->r_eci.x) || std::isnan(slate->v_eci.x))
        {
            LOG_ERROR("Orbit filter state contains NaN at step %d", i);
            passed = false;
            break;
        }

        // Position should stay near Earth radius (within 100 km)
        float r_mag = length(slate->r_eci);
        if (fabsf(r_mag - R_E) > 100.0f)
        {
            LOG_ERROR(
                "Position drifted far from Earth surface: %.3f km at step %d",
                r_mag, i);
            passed = false;
            break;
        }

        // Velocity magnitude should stay constant (Earth rotation)
        // At equator, rotation speed ~ 0.465 km/s
        float v_mag = length(slate->v_eci);
        if (fabsf(v_mag - v_initial_mag) > 0.1f)
        {
            LOG_ERROR(
                "Velocity magnitude changed too much: %.6f km/s at step %d",
                v_mag, i);
            passed = false;
            break;
        }
    }

    float r_final_mag = length(slate->r_eci);
    float v_final_mag = length(slate->v_eci);

    LOG_INFO("Final r_eci: [%.3f, %.3f, %.3f] km (mag=%.3f)", slate->r_eci.x,
             slate->r_eci.y, slate->r_eci.z, r_final_mag);
    LOG_INFO("Final v_eci: [%.6f, %.6f, %.6f] km/s (mag=%.6f)", slate->v_eci.x,
             slate->v_eci.y, slate->v_eci.z, v_final_mag);
    LOG_INFO("Position magnitude drift: %.3f km (%.3f%%)",
             fabsf(r_final_mag - r_initial_mag),
             100.0f * fabsf(r_final_mag - r_initial_mag) / r_initial_mag);
    LOG_INFO("Velocity magnitude change: %.6f km/s (%.3f%%)",
             fabsf(v_final_mag - v_initial_mag),
             100.0f * fabsf(v_final_mag - v_initial_mag) / v_initial_mag);

    LOG_INFO("Test %s", passed ? "PASSED" : "FAILED");
    printf("><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=\n\n");
}

void orbit_filter_convergence_test(slate_t *slate)
{
    printf("\n><=><=><=><=><= Testing Orbit Filter Convergence! "
           "><=><=><=><=><=\n");

    // Initialize with simulated GPS data (ISS-like circular orbit)
    slate->gps_lat = 0.0f;   // degrees (equator for simplicity)
    slate->gps_lon = 0.0f;   // degrees
    slate->gps_alt = 400.0f; // km
    slate->MJD = 60000.0f;   // Some MJD value

    // Compute position from GPS
    float3 r_ecef = lla_to_ecef(slate->gps_lat, slate->gps_lon, slate->gps_alt);
    float3 r_eci_init = ecef_to_eci(r_ecef, slate->MJD);

    // For a circular orbit, compute orbital velocity: v = sqrt(μ/r)
    float r_mag = length(r_eci_init);
    float v_orbital = sqrtf(MU_EARTH / r_mag); // km/s

    // Velocity should be perpendicular to position for circular orbit
    // Choose velocity in the equatorial plane, tangent to the orbit
    // v_eci = v_orbital * (perpendicular direction to r_eci in xy plane)
    float3 r_xy_norm = normalize(float3(r_eci_init.x, r_eci_init.y, 0.0f));
    // Perpendicular in xy plane: rotate 90 degrees
    float3 v_eci_init = v_orbital * float3(-r_xy_norm.y, r_xy_norm.x, 0.0f);

    // Convert velocity back to GPS speed/course for initialization
    // This is approximate - we're essentially reverse-engineering GPS from
    // orbital velocity
    float v_horizontal =
        sqrtf(v_eci_init.x * v_eci_init.x + v_eci_init.y * v_eci_init.y);
    slate->gps_speed = v_horizontal / 0.000514444f; // Convert km/s to knots
    slate->gps_course =
        atan2f(v_eci_init.x, v_eci_init.y) * RAD_TO_DEG; // North = 0
    if (slate->gps_course < 0)
        slate->gps_course += 360.0f;

    orbit_filter_init(slate);

    LOG_INFO("Initialized circular orbit at equator, alt=%.1f km",
             slate->gps_alt);
    LOG_INFO("Orbital velocity: %.3f km/s", v_orbital);
    LOG_INFO("Initial r_eci: [%.3f, %.3f, %.3f] km (mag=%.3f)", slate->r_eci.x,
             slate->r_eci.y, slate->r_eci.z, length(slate->r_eci));
    LOG_INFO("Initial v_eci: [%.3f, %.3f, %.3f] km/s (mag=%.6f)",
             slate->v_eci.x, slate->v_eci.y, slate->v_eci.z,
             length(slate->v_eci));

    int steps = 100;
    float dt = 1.0f; // 1 second propagation steps

    LOG_INFO("Running orbit filter for %d steps (dt=%.1f s, propagate only)",
             steps, dt);

    // Store initial values for comparison
    float r_initial = length(slate->r_eci);
    float v_initial = length(slate->v_eci);

    bool passed = true;
    for (int i = 0; i < steps; i++)
    {
        // Simulate dt by setting last propagate time
        slate->of_last_propagate_time =
            get_absolute_time() - static_cast<uint64_t>(dt * 1e6);

        // Propagate filter (no GPS updates - testing circular orbit stability)
        orbit_filter_propagate(slate);

        // Check for NaN
        if (std::isnan(slate->r_eci.x) || std::isnan(slate->v_eci.x))
        {
            LOG_ERROR("Orbit filter state contains NaN at step %d", i);
            passed = false;
            break;
        }

        // Check orbit magnitude is reasonable (ISS-like ~6700-6800 km)
        float r_mag = length(slate->r_eci);
        if (r_mag < 6400.0f || r_mag > 7000.0f)
        {
            LOG_ERROR("Orbit radius out of bounds: %.3f km at step %d", r_mag,
                      i);
            passed = false;
            break;
        }

        // Check velocity magnitude is reasonable (ISS ~7.66 km/s)
        float v_mag = length(slate->v_eci);
        if (v_mag < 7.0f || v_mag > 8.5f)
        {
            LOG_ERROR("Velocity magnitude out of bounds: %.6f km/s at step %d",
                      v_mag, i);
            passed = false;
            break;
        }
    }

    float r_final_mag = length(slate->r_eci);
    float v_final_mag = length(slate->v_eci);

    // For a circular orbit, radius and velocity magnitude should stay constant
    float r_drift = fabsf(r_final_mag - r_initial);
    float v_drift = fabsf(v_final_mag - v_initial);

    LOG_INFO("Initial orbit radius: %.3f km", r_initial);
    LOG_INFO("Final orbit radius: %.3f km", r_final_mag);
    LOG_INFO("Radius drift: %.3f km (%.3f%%)", r_drift,
             100.0f * r_drift / r_initial);

    LOG_INFO("Initial velocity: %.6f km/s", v_initial);
    LOG_INFO("Final velocity: %.6f km/s", v_final_mag);
    LOG_INFO("Velocity drift: %.6f km/s (%.3f%%)", v_drift,
             100.0f * v_drift / v_initial);

    LOG_INFO("Final P log frobenius: %.6f", slate->P_orbit_log_frobenius);
    LOG_INFO("Final r_eci: [%.3f, %.3f, %.3f] km", slate->r_eci.x,
             slate->r_eci.y, slate->r_eci.z);
    LOG_INFO("Final v_eci: [%.6f, %.6f, %.6f] km/s", slate->v_eci.x,
             slate->v_eci.y, slate->v_eci.z);

    // Check that orbit remained reasonably circular (< 1% drift in 100 seconds)
    if (r_drift / r_initial > 0.01f)
    {
        LOG_ERROR("Orbit radius drifted too much (> 1%%)!");
        passed = false;
    }
    if (v_drift / v_initial > 0.01f)
    {
        LOG_ERROR("Velocity magnitude drifted too much (> 1%%)!");
        passed = false;
    }

    LOG_INFO("Test %s", passed ? "PASSED" : "FAILED");
    printf("><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=\n\n");
}

void orbit_filter_multi_orbit_test(slate_t *slate)
{
    printf("\n><=><=><=><=><= Testing Orbit Filter (Multiple Orbits - "
           "Propagation Only) ><=><=><=><=><=\n");

    // Initialize circular orbit at 400 km altitude
    slate->gps_lat = 0.0f;
    slate->gps_lon = 0.0f;
    slate->gps_alt = 400.0f;
    slate->MJD = 60000.0f;

    // Compute initial circular orbit
    float3 r_ecef = lla_to_ecef(slate->gps_lat, slate->gps_lon, slate->gps_alt);
    float3 r_eci_init = ecef_to_eci(r_ecef, slate->MJD);
    float r_mag = length(r_eci_init);
    float v_orbital = sqrtf(MU_EARTH / r_mag);

    float3 r_xy_norm = normalize(float3(r_eci_init.x, r_eci_init.y, 0.0f));
    float3 v_eci_init = v_orbital * float3(-r_xy_norm.y, r_xy_norm.x, 0.0f);

    float v_horizontal =
        sqrtf(v_eci_init.x * v_eci_init.x + v_eci_init.y * v_eci_init.y);
    slate->gps_speed = v_horizontal / 0.000514444f;
    slate->gps_course = atan2f(v_eci_init.x, v_eci_init.y) * RAD_TO_DEG;
    if (slate->gps_course < 0)
        slate->gps_course += 360.0f;

    orbit_filter_init(slate);

    // ISS orbital period ~ 92.7 minutes = 5562 seconds
    // Test for 3 orbits = ~16686 seconds
    float orbital_period =
        2.0f * 3.14159f * sqrtf(r_mag * r_mag * r_mag / MU_EARTH);
    int num_orbits = 3;
    float total_time = num_orbits * orbital_period;
    int steps = static_cast<int>(total_time); // 1 second timesteps
    float dt = 1.0f;

    LOG_INFO("Orbital period: %.1f seconds (%.2f minutes)", orbital_period,
             orbital_period / 60.0f);
    LOG_INFO("Testing %d orbits = %.1f seconds (%.2f minutes)", num_orbits,
             total_time, total_time / 60.0f);
    LOG_INFO("Propagate every %.1f s (propagation-only, no GPS updates)", dt);
    LOG_INFO("Initial r: %.3f km, v: %.6f km/s", length(slate->r_eci),
             length(slate->v_eci));

    float r_initial = length(slate->r_eci);
    float v_initial = length(slate->v_eci);
    float r_max = r_initial;
    float r_min = r_initial;

    bool passed = true;
    for (int i = 0; i < steps; i++)
    {
        slate->of_last_propagate_time =
            get_absolute_time() - static_cast<uint64_t>(dt * 1e6);

        orbit_filter_propagate(slate);

        // Track min/max radius
        float r_mag_current = length(slate->r_eci);
        if (r_mag_current > r_max)
            r_max = r_mag_current;
        if (r_mag_current < r_min)
            r_min = r_mag_current;

        // Check for NaN
        if (std::isnan(slate->r_eci.x) || std::isnan(slate->v_eci.x))
        {
            LOG_ERROR("State contains NaN at step %d (t=%.1f s)", i, i * dt);
            passed = false;
            break;
        }

        // Check bounds (allow some drift but catch major issues)
        if (r_mag_current < 6400.0f || r_mag_current > 7200.0f)
        {
            LOG_ERROR("Radius out of bounds: %.3f km at step %d", r_mag_current,
                      i);
            passed = false;
            break;
        }

        // Log progress every orbit
        if (i > 0 && i % static_cast<int>(orbital_period) == 0)
        {
            int orbit_num = i / static_cast<int>(orbital_period);
            LOG_INFO("Orbit %d complete: r=%.3f km, v=%.6f km/s", orbit_num + 1,
                     r_mag_current, length(slate->v_eci));
        }
    }

    float r_final = length(slate->r_eci);
    float v_final = length(slate->v_eci);
    float eccentricity = (r_max - r_min) / (r_max + r_min);

    LOG_INFO("=== Final Results ===");
    LOG_INFO("Initial radius: %.3f km", r_initial);
    LOG_INFO("Final radius: %.3f km", r_final);
    LOG_INFO("Min radius: %.3f km", r_min);
    LOG_INFO("Max radius: %.3f km", r_max);
    LOG_INFO("Eccentricity estimate: %.6f (should be ~0 for circular)",
             eccentricity);
    LOG_INFO("Radius drift: %.3f km (%.3f%%)", fabsf(r_final - r_initial),
             100.0f * fabsf(r_final - r_initial) / r_initial);
    LOG_INFO("Velocity drift: %.6f km/s (%.3f%%)", fabsf(v_final - v_initial),
             100.0f * fabsf(v_final - v_initial) / v_initial);
    LOG_INFO("Final P log frobenius: %.6f", slate->P_orbit_log_frobenius);

    // Check orbit stayed reasonably circular over multiple orbits
    if (fabsf(r_final - r_initial) / r_initial > 0.05f)
    {
        LOG_ERROR("Orbit drifted too much over %d orbits (> 5%%)!", num_orbits);
        passed = false;
    }

    LOG_INFO("Test %s", passed ? "PASSED" : "FAILED");
    printf("><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=\n\n");
}

void orbit_filter_multi_orbit_gps_test(slate_t *slate)
{
    printf("\n><=><=><=><=><= Testing Orbit Filter (Multiple Orbits with GPS) "
           "><=><=><=><=><=\n");

    // *** CONFIGURE NUMBER OF ORBITS HERE ***
    int num_orbits = 50;

    // Initialize circular orbit at 400 km altitude
    slate->gps_lat = 0.0f;
    slate->gps_lon = 0.0f;
    slate->gps_alt = 400.0f;
    slate->MJD = 60000.0f;

    // Compute initial circular orbit
    float3 r_ecef = lla_to_ecef(slate->gps_lat, slate->gps_lon, slate->gps_alt);
    float3 r_eci_init = ecef_to_eci(r_ecef, slate->MJD);
    float r_mag = length(r_eci_init);
    float v_orbital = sqrtf(MU_EARTH / r_mag);

    float3 r_xy_norm = normalize(float3(r_eci_init.x, r_eci_init.y, 0.0f));
    float3 v_eci_init = v_orbital * float3(-r_xy_norm.y, r_xy_norm.x, 0.0f);

    float v_horizontal =
        sqrtf(v_eci_init.x * v_eci_init.x + v_eci_init.y * v_eci_init.y);
    slate->gps_speed = v_horizontal / 0.000514444f;
    slate->gps_course = atan2f(v_eci_init.x, v_eci_init.y) * RAD_TO_DEG;
    if (slate->gps_course < 0)
        slate->gps_course += 360.0f;

    orbit_filter_init(slate);

    // Orbital period calculation
    float orbital_period =
        2.0f * 3.14159f * sqrtf(r_mag * r_mag * r_mag / MU_EARTH);
    float total_time = num_orbits * orbital_period;
    int steps = static_cast<int>(total_time); // 1 second timesteps
    float dt = 1.0f;
    int gps_update_interval = 30; // GPS update every 30 seconds

    LOG_INFO("Orbital period: %.1f seconds (%.2f minutes)", orbital_period,
             orbital_period / 60.0f);
    LOG_INFO("Testing %d orbits = %.1f seconds (%.2f minutes)", num_orbits,
             total_time, total_time / 60.0f);
    LOG_INFO("Propagate every %.1f s, GPS update every %d s", dt,
             gps_update_interval);
    LOG_INFO("Initial r: %.3f km, v: %.6f km/s", length(slate->r_eci),
             length(slate->v_eci));

    float r_initial = length(slate->r_eci);
    float v_initial = length(slate->v_eci);
    float r_max = r_initial;
    float r_min = r_initial;

    // True orbit state (independent from filter for realistic GPS simulation)
    float x_true[6] = {slate->r_eci.x, slate->r_eci.y, slate->r_eci.z,
                       slate->v_eci.x, slate->v_eci.y, slate->v_eci.z};

    bool passed = true;
    for (int i = 0; i < steps; i++)
    {
        // Update MJD as simulation progresses (dt in seconds, MJD in days)
        slate->MJD = 60000.0f + (i * dt) / 86400.0f;

        slate->of_last_propagate_time =
            get_absolute_time() - static_cast<uint64_t>(dt * 1e6);

        orbit_filter_propagate(slate);

        // Propagate true orbit independently (with zero IMU acceleration)
        float x_true_new[6];
        float3 a_imu_zero = {0.0f, 0.0f, 0.0f};
        rk4_step(x_true_new, x_true, dt, a_imu_zero, slate->q_eci_to_body);
        for (int j = 0; j < 6; j++)
            x_true[j] = x_true_new[j];

        // Simulate GPS update every 30 seconds using TRUE orbit state
        if (i % gps_update_interval == 0)
        {
            float3 r_true = {x_true[0], x_true[1], x_true[2]};
            float3 v_true = {x_true[3], x_true[4], x_true[5]};

            // Convert true ECI -> ECEF -> LLA for GPS measurement
            float3 r_ecef_meas = eci_to_ecef(r_true, slate->MJD);
            float3 lla_meas = ecef_to_lla(r_ecef_meas);

            // Convert true ECI velocity -> ECEF -> ENU -> speed/course
            float3 v_ecef_meas = eci_to_ecef(v_true, slate->MJD);
            float3 v_enu_meas = ecef_to_enu(v_ecef_meas, lla_meas);

            // Update slate with simulated GPS measurement from TRUE orbit
            slate->gps_lat = lla_meas.x;
            slate->gps_lon = lla_meas.y;
            slate->gps_alt = lla_meas.z;

            float v_horizontal_meas = sqrtf(v_enu_meas.x * v_enu_meas.x +
                                            v_enu_meas.y * v_enu_meas.y);
            slate->gps_speed =
                v_horizontal_meas / 0.000514444f; // km/s to knots
            slate->gps_course = atan2f(v_enu_meas.x, v_enu_meas.y) * RAD_TO_DEG;
            if (slate->gps_course < 0)
                slate->gps_course += 360.0f;

            orbit_filter_update(slate);
        }

        // Track min/max radius
        float r_mag_current = length(slate->r_eci);
        if (r_mag_current > r_max)
            r_max = r_mag_current;
        if (r_mag_current < r_min)
            r_min = r_mag_current;

        // Check for NaN
        if (std::isnan(slate->r_eci.x) || std::isnan(slate->v_eci.x))
        {
            LOG_ERROR("State contains NaN at step %d (t=%.1f s)", i, i * dt);
            passed = false;
            break;
        }

        // Check bounds (allow some drift but catch major issues)
        if (r_mag_current < 6400.0f || r_mag_current > 7200.0f)
        {
            LOG_ERROR("Radius out of bounds: %.3f km at step %d", r_mag_current,
                      i);
            passed = false;
            break;
        }

        // Log progress every orbit
        if (i > 0 && i % static_cast<int>(orbital_period) == 0)
        {
            int orbit_num = i / static_cast<int>(orbital_period);
            LOG_INFO("Orbit %d complete: r=%.3f km, v=%.6f km/s, P_log=%.3f",
                     orbit_num + 1, r_mag_current, length(slate->v_eci),
                     slate->P_orbit_log_frobenius);
        }
    }

    float r_final = length(slate->r_eci);
    float v_final = length(slate->v_eci);
    float eccentricity = (r_max - r_min) / (r_max + r_min);

    LOG_INFO("=== Final Results ===");
    LOG_INFO("Initial radius: %.3f km", r_initial);
    LOG_INFO("Final radius: %.3f km", r_final);
    LOG_INFO("Min radius: %.3f km", r_min);
    LOG_INFO("Max radius: %.3f km", r_max);
    LOG_INFO("Eccentricity estimate: %.6f (should be ~0 for circular)",
             eccentricity);
    LOG_INFO("Radius drift: %.3f km (%.3f%%)", fabsf(r_final - r_initial),
             100.0f * fabsf(r_final - r_initial) / r_initial);
    LOG_INFO("Velocity drift: %.6f km/s (%.3f%%)", fabsf(v_final - v_initial),
             100.0f * fabsf(v_final - v_initial) / v_initial);
    LOG_INFO("Final P log frobenius: %.6f", slate->P_orbit_log_frobenius);

    // With GPS updates, should stay very close to circular
    if (fabsf(r_final - r_initial) / r_initial > 0.01f)
    {
        LOG_ERROR("Orbit drifted too much with GPS (> 1%%)!");
        passed = false;
    }

    LOG_INFO("Test %s", passed ? "PASSED" : "FAILED");
    printf("><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=><=\n\n");
}
#endif
