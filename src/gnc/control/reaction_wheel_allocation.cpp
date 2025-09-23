/**
 * @author Sidharth Anantha
 * @date 2025-02-08
 */

#include "reaction_wheel_allocation.h"
#include "bdot.h"
#include "constants.h"
#include "linalg.h"
#include "macros.h"
#include "pico/stdlib.h"
#include <cmath>

// Define constants:
// PID gains for the reaction wheel control:
constexpr float K_P = 1;          // [N*m/rad]   CHANGE THIS
constexpr float K_D = 1;          // [N*m/rad/s] CHANGE THIS
constexpr float MAX_TORQUE = 1e3; // [N*m]      CHANGE THIS

linalg::vec<float, 3> compute_control_torque(
    linalg::vec<float, 4> q_current, linalg::vec<float, 4> q_desired,
    linalg::vec<float, 3> omega_current, linalg::vec<float, 3> omega_desired)
{
    // Now we can work directly with the quaternions and angular velocities
    q_current = linalg::normalize(q_current);
    q_desired = linalg::normalize(q_desired);

    // Compute Axis Angle:
    // Get the angle between quaternions
    float angle = linalg::angle(q_current, q_desired);
    // First get the difference quaternion
    linalg::vec<float, 4> diff_quat =
        linalg::qmul(q_current, linalg::qconj(q_desired));
    // Then extract the axis
    linalg::vec<float, 3> axis = linalg::qaxis(diff_quat);

    // Compute the control torque:
    linalg::vec<float, 3> tau =
        K_P * axis * angle + K_D * (omega_desired - omega_current);

    // Need to check if the torque is beyond wheel max torque:
    float tau_mag = linalg::length(tau);
    if (tau_mag > MAX_TORQUE)
    {
        // Normalize the torque vector:
        tau = (tau * MAX_TORQUE) / tau_mag; // [N*m]
    }

    return tau;
}

linalg::vec<float, 4> allocate_reaction_wheels(
    linalg::vec<float, 4> q_current, linalg::vec<float, 4> q_desired,
    linalg::vec<float, 3> omega_current, linalg::vec<float, 3> omega_desired)
{
    // Now we can directly use the quaternions and angular velocities
    linalg::vec<float, 3> tau = compute_control_torque(
        q_current, q_desired, omega_current, omega_desired);
    float motor_MOI = 7.90e-7; // kg*m^2

    // Define the W_pseudoinv matrix
    // This was pre-computed from Python
    // Assumes a tetrahedron configuration, based only on geometry
    // CHANGE THIS IF YOU CHANGE THE GEOMETRY CONFIGURATION
    linalg::mat<float, 4, 3> W_pseudoinv = {
        {-0.1005f, 0.3108f, -0.3061f, -0.3061f}, // first column
        {0.0f, 0.0f, 0.6124f, -0.6124f},         // second column
        {0.7843f, -0.3562f, -0.1454f, -0.1454f}  // third column
    };

    // Compute the reaction wheel torques:
    linalg::vec<float, 4> wheel_torque = linalg::mul(W_pseudoinv, tau); // [N*m]

    // Compute the reaction wheel speeds:
    linalg::vec<float, 4> wheel_speeds = wheel_torque / motor_MOI; // [rad/s]

    return wheel_speeds; // vector dim=4 for all wheels
}