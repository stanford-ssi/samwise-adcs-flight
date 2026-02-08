/*
 * @author Lundeen Cahilly
 * @date 2026-02-04
 *
 * This file implements an attitude propagator for testing purposes.
 */

#include "attitude_propagator.h"
#include "constants.h"

// Forward declarations
static void q_dot(float4 &q_dot, float3 &w_dot, float4 q, float3 w,
                  float3 tau_body, float dt);
static void rk4_step(float4 &q_new, float3 &w_new, float3 tau_body, float dt);
static float3 tau_gravity_gradient(float4 q_eci2body, float3 r_eci);
static float3 tau_drag(float4 q_eci2body, float3 r_eci, float3 v_eci);
static float3 tau_magnetorquer(float4 q_eci2body, float3 b_eci, float3 mu_mt);

/* This function propagates the attitude of the satellite given the current
 * ground truth state vector, actuator control inputs, and a time step dt. The
 * attitude is propagated using an RK4 integration scheme.
 *
 * @param q_eci2body Attitude quaternion from ECI to body frame
 * @param w_eci Angular velocity in body frame [rad/s]
 * @param r_eci Position vector in ECI frame [km]
 * @param v_eci Velocity vector in ECI frame [km/s]
 * @param b_eci Magnetic field vector in ECI frame [nT]
 * @param mu_mt Magnetic moment of the satellite [A*m^2]
 * @param tau_rw Reaction wheel torque in body frame [Nm]
 * @param dt Time step [s]
 */
void propagate_attitude(float4 &q_eci2body, float3 &w_eci, float3 r_eci,
                        float3 v_eci, float3 b_eci, float3 mu_mt, float3 tau_rw,
                        float dt)
{
    // Get overall torque on satellite in body frame
    float3 tau_gg = tau_gravity_gradient(q_eci2body, r_eci);
    float3 tau_d = tau_drag(q_eci2body, r_eci, v_eci);
    float3 tau_mt = tau_magnetorquer(q_eci2body, b_eci, mu_mt);
    float3 tau_body =
        tau_gg + tau_mt +
        tau_rw; // TODO: add area projection to drag torque, then add here

    // Propagate attitude using RK4 integrator
    rk4_step(q_eci2body, w_eci, tau_body, dt);
}

/*
 * This function performs a single RK4 integration step for the attitude of the
 * satellite.
 *
 * @param q_new New quaternion in ECI to body frame
 * @param w_new New angular velocity in body frame [rad/s]
 * @param tau_body Total torque in body frame [Nm]
 * @param dt time step [s]
 */
void rk4_step(float4 &q_new, float3 &w_new, float3 tau_body, float dt)
{
    // rK4 integration
    float4 k1_q, k2_q, k3_q, k4_q; // quaternion derivatives
    float3 k1_w, k2_w, k3_w, k4_w; // angular velocity derivatives
    float4 q_temp;
    float3 w_temp;

    // k1 = f(q, w, tau_body, dt) = [q_dot, w_dot]
    q_dot(k1_q, k1_w, q_new, w_new, tau_body, dt);

    // k2 = f(q + dt/2 * k1, w + dt/2 * k1)
    q_temp = q_new + 0.5f * dt * k1_q;
    q_temp = normalize(q_temp);
    w_temp = w_new + 0.5f * dt * k1_w;
    q_dot(k2_q, k2_w, q_temp, w_temp, tau_body, dt);

    // k3 = f(q + dt/2 * k2, w + dt/2 * k2)
    q_temp = q_new + 0.5f * dt * k2_q;
    q_temp = normalize(q_temp);
    w_temp = w_new + 0.5f * dt * k2_w;
    q_dot(k3_q, k3_w, q_temp, w_temp, tau_body, dt);

    // k4 = f(q + dt * k3, w + dt * k3)
    q_temp = q_new + dt * k3_q;
    q_temp = normalize(q_temp);
    w_temp = w_new + dt * k3_w;
    q_dot(k4_q, k4_w, q_temp, w_temp, tau_body, dt);

    // q_new = q + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    q_new = q_new + (dt / 6.0f) * (k1_q + 2.0f * k2_q + 2.0f * k3_q + k4_q);
    q_new = normalize(q_new);

    // w_new = w + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    w_new = w_new + (dt / 6.0f) * (k1_w + 2.0f * k2_w + 2.0f * k3_w + k4_w);
}

/* This function computes the quaternion and angular velocity derivatives.
 *
 * @param q_dot Quaternion derivative
 * @param w_dot Angular velocity derivative
 * @param q Quaternion
 * @param w Angular velocity
 * @param tau_body Total torque in body frame [Nm]
 * @param dt Time step [s]
 */
void q_dot(float4 &q_dot_out, float3 &w_dot, float4 q, float3 w,
           float3 tau_body, float dt)
{
    // Compute quaternion derivative: q_dot = 0.5 * q ⊗ [w, 0]
    float4 w_quat = {w.x, w.y, w.z, 0.0f};
    q_dot_out = 0.5f * qmul(q, w_quat);

    // Compute angular velocity derivative using Euler's equations:
    // w_dot = I^-1 * (tau - w × (I * w))
    float3 I_w = mul(I_BODY, w);
    float3 gyroscopic = cross(w, I_w);
    w_dot = mul(inverse(I_BODY), tau_body - gyroscopic);
}

/* This function computes the gravity gradient torque on the satellite in the
 * body frame. tau_gg = -3 mu / r^5 * r_body x (I_body r_body)
 *
 * @param q_eci2body Attitude quaternion from ECI to body frame
 * @param r_eci Position vector in ECI frame [km]
 * @return gravity gradient torque in body frame [Nm]
 */
static float3 tau_gravity_gradient(float4 q_eci2body, float3 r_eci)
{
    // Convert km to m for calculations
    float mu = MU_EARTH * 1e9f; // [km^3/s^2] -> [m^3/s^2]
    r_eci = r_eci * 1e3f;       // [km] -> [m]

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
 * TODO: do not assume constant area (this will require convex hull or lookup
 * table)
 * TODO: assume Cp at [0, 0, 0] for now
 *
 * @param q_eci2body Attitude quaternion from ECI to body frame
 * @param r_eci Position vector in ECI frame [km]
 * @param v_eci Velocity vector in ECI frame [km/s]
 * @return drag torque in body frame [Nm]
 */
static float3 tau_drag(float4 q_eci2body, float3 r_eci, float3 v_eci)
{
    // Constants
    float RHO_ATM = 1.225f; // [kg/m^3] TODO: use actual atmospheric density
    float cd = 2.2f;        // TODO: use actual drag coefficient
    float area = 0.0345f; // [m^2] TODO: do not assume constant area (this will
                          // require convex hull or lookup table)

    // Convert km to m for calculations
    r_eci = r_eci * 1e3f; // [km] -> [m]
    v_eci = v_eci * 1e3f; // [km/s] -> [m/s]

    // Find atmospheric velocity assuming constant rotation rate of Earth
    float3 w_earth_vec = {0.0f, 0.0f,
                          W_EARTH}; // Earth rotation vector around z-axis
    float3 v_atm = v_eci - cross(w_earth_vec, r_eci);

    // Rotate v_atm to body frame
    float3 v_atm_body = qrot(q_eci2body, v_atm);

    // Calculate drag force: F = 0.5 * rho * Cd * A * |v| * v
    float v_mag = length(v_atm_body);
    float3 F_drag = -0.5f * RHO_ATM * cd * area * v_mag * v_atm_body; // [N]

    // Get drag torque
    float3 r_cp2cm =
        R_CP - R_CM; // [m] distance from center of pressure to center of mass
    float3 tau_drag = cross(r_cp2cm, F_drag); // torque due to drag

    return tau_drag;
}

/* This function calculates the magnetorquer torque on the satellite in the body
 * frame. tau_mt = mu_mag x b_body
 *
 * @param q_eci2body Attitude quaternion from ECI to body frame
 * @param b_eci Magnetic field vector in ECI frame [nT]
 * @param mu_mag Magnetic moment of the satellite [A*m^2]
 * @return magnetorquer torque in body frame [Nm]
 */
static float3 tau_magnetorquer(float4 q_eci2body, float3 b_eci, float3 mu_mt)
{
    // Convert nT to T for calculations
    b_eci = b_eci * 1e-9f; // [nT] -> [T]

    // Convert b_eci to body frame
    float3 b_body = qrot(q_eci2body, b_eci);

    float3 tau_mt = cross(mu_mt, b_body);
    return tau_mt;
}

#ifdef TEST
void test_torque_free_motion();

void test_propagator()
{
    test_torque_free_motion();
}

// torque free motion test
void test_torque_free_motion()
{
    LOG_INFO("=>=>=> Testing torque free motion... <=<=<=");
    bool passed = true;
    float4 q_eci2body = {0, 0, 0, 1};
    float3 w_body = {0.01f, 0.02f, 0.1f}; // [rad/s] - tumbling motion
    float dt = 0.01f;                     // [s]
    int steps = 10000;

    // Compute initial angular momentum in BODY frame
    float3 L_body_initial = mul(I_BODY, w_body);

    // Convert to INERTIAL frame (this should stay constant!)
    float3 L_inertial_initial = qrot(q_eci2body, L_body_initial);

    // Compute initial energy
    float E_initial = 0.5f * dot(w_body, L_body_initial);

    // Propagate attitude
    for (int i = 0; i < steps; i++)
    {
        rk4_step(q_eci2body, w_body, {0.0f, 0.0f, 0.0f}, dt);
    }

    // Check L in inertial frame (should be conserved!)
    float3 L_body_final = mul(I_BODY, w_body);
    float3 L_inertial_final = qrot(q_eci2body, L_body_final);

    // Check energy conservation
    float E_final = 0.5f * dot(w_body, L_body_final);

    // Check quaternion normalization
    float q_norm = length(q_eci2body);

    LOG_INFO("L_inertial_initial: %f %f %f", L_inertial_initial.x,
             L_inertial_initial.y, L_inertial_initial.z);
    LOG_INFO("L_inertial_final:   %f %f %f", L_inertial_final.x,
             L_inertial_final.y, L_inertial_final.z);
    LOG_INFO("L_body_initial: %f %f %f", L_body_initial.x, L_body_initial.y,
             L_body_initial.z);
    LOG_INFO("L_body_final:   %f %f %f", L_body_final.x, L_body_final.y,
             L_body_final.z);
    LOG_INFO("E_initial: %.9f, E_final: %.9f, dE: %.9e", E_initial, E_final,
             E_final - E_initial);
    LOG_INFO("q_norm: %.9f", q_norm);
    LOG_INFO("w_final: %f %f %f", w_body.x, w_body.y, w_body.z);

    if (length(L_inertial_final - L_inertial_initial) > 1e-5f)
    {
        LOG_ERROR("Angular momentum not conserved in inertial frame! dL = %.9e",
                  length(L_inertial_final - L_inertial_initial));
        passed = false;
    }
    if (fabs(E_final - E_initial) > 1e-6f)
    {
        LOG_ERROR("Energy not conserved! dE = %.9e", E_final - E_initial);
        passed = false;
    }
    if (fabs(q_norm - 1.0f) > 1e-5f)
    {
        LOG_ERROR("Quaternion not normalized! |q| = %.9f", q_norm);
        passed = false;
    }

    LOG_INFO("=>=>=> Test %s <=<=<=\n", passed ? "PASSED" : "FAILED");
}

void tau_gravity_gradient_test()
{
    // TODO: add more tests
    LOG_INFO("=>=>=> Testing gravity gradient torque... <=<=<=");
    bool passed = true; // TODO: Add actual test
    float4 q_eci2body = {cosf(45.0f / 2.0f), 0, 0, sinf(45.0f / 2.0f)};
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
    float3 v_eci = {0.0f, 0.0f, 0.0f};      // [km/s]
    float3 tau_d = tau_drag(q_eci2body, r_eci, v_eci);
    LOG_INFO("tau_d: %f %f %f", tau_d[0], tau_d[1], tau_d[2]);
    LOG_INFO("=>=>=> Test %s <=<=<=\n", passed ? "PASSED" : "FAILED");
}

void tau_magnetorquer_test()
{
    LOG_INFO("=>=>=> Testing magnetorquer torque... <=<=<=");
    bool passed = true;
    float4 q_eci2body = {0.70710678f, 0, 0, 0.70710678f}; // 90 deg about X
    float3 b_eci = {0.0f, 50000.0f, 0.0f};                // [nT]
    float3 mu_mag = {0.1f, 0.0f,
                     0.0f}; // [A*m^2] perpendicular to b_eci after rotation
    float3 tau_mt = tau_magnetorquer(q_eci2body, b_eci, mu_mag);
    LOG_INFO("tau_mt: %f %f %f", tau_mt[0], tau_mt[1],
             tau_mt[2]); // should be in negative y direction
    LOG_INFO("=>=>=> Test %s <=<=<=\n", passed ? "PASSED" : "FAILED");
}
#endif