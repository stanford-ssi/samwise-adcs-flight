/**
 * @author Sidharth Anantha
 * @date 2025-02-08
 */

#include "reaction_wheel_allocation.h"
#include "constants.h"
#include "gnc/control/bdot.h"
#include "linalg.h"
#include "macros.h"
#include "pico/stdlib.h"
#include <cmath>

using namespace linalg::aliases;

// TODO: This file is a rough draft and must be refactored
// before other files are connected to it

// Define constants:
// PID gains for the reaction wheel control:
constexpr float K_P = 1;          // [N*m/rad]   CHANGE THIS
constexpr float K_D = 1;          // [N*m/rad/s] CHANGE THIS
constexpr float MAX_TORQUE = 1e3; // [N*m]      CHANGE THIS

/**
 * Compute control torque using PD control based on quaternion error and angular
 * velocity error
 *
 * @param q_current Current attitude quaternion
 * @param q_desired Desired attitude quaternion
 * @param w_current Current angular velocity
 * @param w_desired Desired angular velocity
 * @return Control torque vector (clamped to MAX_TORQUE)
 */
float3 compute_control_torque(float4 q_current, float4 q_desired,
                              float3 w_current, float3 w_desired)
{
    // Now we can work directly with the quaternions and angular velocities
    q_current = normalize(q_current);
    q_desired = normalize(q_desired);

    // Compute axis angle representation
    // Get the angle between quaternions
    float angle = linalg::angle(q_current, q_desired);
    // First get the difference quaternion
    float4 diff_quat = qmul(q_current, qconj(q_desired));
    // Then extract the axis
    float3 axis = qaxis(diff_quat);

    // Compute the control torque
    float3 tau = K_P * axis * angle + K_D * (w_desired - w_current);

    // Clamp torque to maximum allowable value
    float tau_mag = length(tau);
    if (tau_mag > MAX_TORQUE)
    {
        // Normalize the torque vector
        tau = (tau * MAX_TORQUE) / tau_mag; // [N*m]
    }

    return tau;
}

/**
 * Allocate reaction wheel speeds to achieve desired attitude control
 *
 * @param q_current Current attitude quaternion
 * @param q_desired Desired attitude quaternion
 * @param w_current Current angular velocity
 * @param w_desired Desired angular velocity
 * @return Reaction wheel speeds for tetrahedral configuration
 */
float4 allocate_reaction_wheels(float4 q_current, float4 q_desired,
                                float3 w_current, float3 w_desired)
{
    // Now we can directly use the quaternions and angular velocities
    float3 tau =
        compute_control_torque(q_current, q_desired, w_current, w_desired);
    float motor_MOI = 7.90e-7; // kg*m^2

    // Define the W_pseudoinv matrix
    // This was pre-computed from Python
    // Assumes a tetrahedron configuration, based only on geometry
    // CHANGE THIS IF YOU CHANGE THE GEOMETRY CONFIGURATION
    float4x3 W_pseudoinv = {
        {-0.1005f, 0.3108f, -0.3061f, -0.3061f}, // first column
        {0.0f, 0.0f, 0.6124f, -0.6124f},         // second column
        {0.7843f, -0.3562f, -0.1454f, -0.1454f}  // third column
    };

    // Compute the reaction wheel torques
    float4 wheel_torque = mul(W_pseudoinv, tau); // [N*m]

    // Compute the reaction wheel speeds
    float4 wheel_speeds = wheel_torque / motor_MOI; // [rad/s]

    return wheel_speeds;
}
