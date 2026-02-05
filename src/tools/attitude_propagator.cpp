/*
 * @author Lundeen Cahilly
 * @date 2026-02-04
 *
 * This file implements an attitude propagator for testing purposes.
 */

#include "attitude_propagator.h"
#include "constants.h"

// Forward declarations
static float3 tau_gravity_gradient(float4 q_eci2body, float3 r_eci);
static float atmospheric_density(float3 r_eci);

/* This function propagates the attitude of the satellite given the current
 * ground truth state vector, actuator control inputs, and a time step dt. The
 * attitude is propagated using an RK4 integration scheme.
 * 
 * @param q_eci2body Attitude quaternion from ECI to body frame
 * @param w_eci Angular velocity in body frame [rad/s]
 * @param r_eci Position vector in ECI frame [km]
 * @param v_eci Velocity vector in ECI frame [km/s]
 * @param mu_mag Magnetic moment of the satellite [A*m^2]
 * @param tau_rw Reaction wheel torque in body frame [Nm]
 * @param dt Time step [s]
 */
void propagate_attitude(float4 &q_eci2body, float3 &w_eci, float3 r_eci, float3 v_eci,
                        float3 mu_mag, float3 tau_rw, float dt)
{
    // Get overall torque on satellite in body frame
    float3 tau_gg = tau_gravity_gradient(q_eci2body, r_eci);
    float3 tau_body = tau_gg; // TODO: Add drag, reaction wheel, and magnetorquer torques

    // Propagate attitude using RK4 integrator
    float4 q_new = q_eci2body;
    float3 w_new = w_eci;
}

/* This function computes the gravity gradient torque on the satellite in the body frame.
 * tau_gg = -3 mu / r^5 * r_body x (I_body r_body)
 * 
 * @param q_eci2body Attitude quaternion from ECI to body frame
 * @param r_eci Position vector in ECI frame [km]
 * @return gravity gradient torque in body frame [Nm]
 */
static float3 tau_gravity_gradient(float4 q_eci2body, float3 r_eci)
{
    // Convert km to m for calculations
    float mu = MU_EARTH * 1e9f; // [km^3/s^2] -> [m^3/s^2]
    r_eci = r_eci * 1e3f; // [km] -> [m]

    // Convert r_eci to body frame
    float3 r_body = qrot(q_eci2body, r_eci);

    // Compute constant term
    float r_mag = length(r_eci);
    float r5 = r_mag * r_mag * r_mag * r_mag * r_mag;
    float constant = 3 * mu / r5;

    // Compute the gravity gradient torque
    float3 I_r_body = mul(I_BODY, r_body);
    float3 tau_gg = constant * cross(r_body, I_r_body);
    return tau_gg;
}

/* This function calculates the drag force on the satellite in the body frame.
 * F_drag = 0.5 * rho * Cd * A * v^2
 * tau_drag = r_cp2cm x F_drag
 * TODO: do not assume constant area (this will require convex hull or lookup table)
 * TODO: assume Cp at [0, 0, 0] for now
 * 
 * @param q_eci2body Attitude quaternion from ECI to body frame
 * @param r_eci Position vector in ECI frame [km]
 * @param v_eci Velocity vector in ECI frame [km/s]
 * @return drag torque in body frame [Nm]
*/
static float3 tau_drag(float4 q_eci2body, float3 r_eci, float3 v_eci) {
    // Constants 
    float RHO_ATM = 1.225f; // [kg/m^3] TODO: use actual atmospheric density
    float cd = 2.2f; // TODO: use actual drag coefficient
    float area = 0.0345f; // [m^2] TODO: do not assume constant area (this will require convex hull or lookup table)

    // Convert km to m for calculations
    r_eci = r_eci * 1e3f; // [km] -> [m]
    v_eci = v_eci * 1e3f; // [km/s] -> [m/s]

    // Find atmospheric velocity assuming constant rotation rate of Earth
    float3 w_earth_vec = {0.0f, 0.0f, W_EARTH}; // Earth rotation vector around z-axis
    float3 v_atm = v_eci - cross(w_earth_vec, r_eci);

    // Rotate v_atm to body frame
    float3 v_atm_body = qrot(q_eci2body, v_atm);

    // Calculate drag force: F = 0.5 * rho * Cd * A * |v| * v
    float v_mag = length(v_atm_body);
    float3 F_drag = -0.5f * RHO_ATM * cd * area * v_mag * v_atm_body; // [N]

    // Get drag torque
    float3 r_cp2cm = R_CP - R_CM; // [m] distance from center of pressure to center of mass
    float3 tau_drag = cross(r_cp2cm, F_drag); // torque due to drag

    return tau_drag;
}

#ifdef TEST

/* This function tests the gravity gradient torque computation.
 */
void tau_gravity_gradient_test()
{
    // TODO: add more tests
    LOG_INFO("=>=>=> Testing gravity gradient torque... <=<=<=");
    bool passed = true; // TODO: Add actual test
    float4 q_eci2body = {cosf(45.0f/2.0f), 0, 0, sinf(45.0f/2.0f)};
    float3 r_eci = {6378.137f, 0.0f, 0.0f}; // equator, prime meridian
    float3 tau_gg = tau_gravity_gradient(q_eci2body, r_eci);
    LOG_INFO("tau_gg: %.12f %.12f %.12f", tau_gg[0], tau_gg[1], tau_gg[2]);
    LOG_INFO("=>=>=> Test %s <=<=<=\n", passed ? "PASSED" : "FAILED");
}

void tau_drag_test()
{
    LOG_INFO("=>=>=> Testing drag torque... <=<=<=");
    bool passed = true; // TODO: Add actual test
    float4 q_eci2body = {0.70710678f, 0, 0, 0.70710678f}; // 90 deg about X
    float3 r_eci = {6378.137f, 0.0f, 0.0f}; // Equator at prime meridian
    float3 v_eci = {0.0f, 0.0f, 0.0f}; // [km/s]
    float3 tau_d = tau_drag(q_eci2body, r_eci, v_eci);
    LOG_INFO("tau_d: %f %f %f", tau_d[0], tau_d[1], tau_d[2]);
    LOG_INFO("=>=>=> Test %s <=<=<=\n", passed ? "PASSED" : "FAILED");
}

#endif