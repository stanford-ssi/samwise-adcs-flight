/**
 * @author The ADCS team
 * @date 2025-02-08
 */

#include "constants.h"
#include "gnc/bdot.h"
#include "linalg.h"
#include "macros.h"
#include "pico/stdlib.h"

// Define constants:
// PID gains for the reaction wheel control:
constexpr float K_P = 1;          // [N*m/rad]   CHANGE THIS
constexpr float K_D = 1;          // [N*m/rad/s] CHANGE THIS
constexpr float MAX_TORQUE = 1e3; // [N*m]      CHANGE THIS

linalg::vec<float, 3> compute_control_torque(linalg::vec<float, 4> x_current,
                                             linalg::vec<float, 4> x_desired)
{
    // Extract the current and desired quaternions from the state vector:
    linalg::vec<float, 4> q_current =
        normalize({x_current[0], x_current[1], x_current[2], x_current[3]});
    linalg::vec<float, 4> q_desired =
        normalize({x_desired[0], x_desired[1], x_desired[2], x_desired[3]});

    // Compute Axis Angle:
    // Get the angle between quaternions
    float angle = linalg::angle(quat1, quat2);
    // First get the difference quaternion
    linalg::vec<float, 4> diff_quat = linalg::qmul(quat2, linalg::qconj(quat1));
    // Then extract the axis
    linalg::vec<float, 3> axis = linalg::qaxis(diff_quat);

    // Extract angulae valocities:
    linalg::vec<float, 3> omega_current = {x_current[4], x_current[5],
                                           x_current[6]};
    linalg::vec<float, 3> omega_desired = {x_desired[4], x_desired[5],
                                           x_desired[6]};

    // Compute the control torque:
    linalg::vec<float, 3> tau =
        K_P * axis * angle + K_D * (omega_desired - omega_current);

    // Need to check if the torque is beyond wheel max torque:
    float tau_mag = linalg::norm(tau);
    if (tau_mag > MAX_TORQUE)
    {
        // Normalize the torque vector:
        tau = (tau * MAX_TORQUE) / tau_mag; // [N*m]
    }

    return tau;
}

linalg::vec<float, 4> allocate_reaction_wheels(linalg::vec<float, 4> x_current,
                                               linalg::vec<float, 4> x_desired)
{
    linalg::vec<float, 3> tau = compute_control_torque(x_current, x_desired);

    float motor_MOI = 7.90e-7; // kg*m^2

    // Define the W_pseudoinv matrix
    // This was pre-computed from Python
    // Assumes a tetrahedron configuration, based only on geometry
    // CHANGE THIS IF YOU CHANGE THE GEOMETRY CONFIGURATION
    float W_pseudoinv[3][4] = {{-0.1005, 0, 0.7843},
                               {0.3108, 0, -0.3562},
                               {-0.3061, 0.6124, -0.1454},
                               {-0.3061, -0.6124, -0.1454}};

    // Compute the reaction wheel torques:
    linalg::vec<float, 4> wheel_torque = linalg::mul(W_pseudoinv, tau); // [N*m]

    // Compute the reaction wheel speeds:
    linalg::vec<float, 4> wheel_speeds = wheel_torque / motor_MOI; // [rad/s]

    return wheel_speeds; // vector dim=4 for all wheels
}
