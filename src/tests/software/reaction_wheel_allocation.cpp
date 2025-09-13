/**
 * @author Sidharth Anantha
 * @date 2025-02-08
 */

#include "reaction_wheel_allocation.h"
#include "gnc/control/reaction_wheel_allocation.h"
#include "linalg.h"
#include "macros.h"

using namespace linalg::aliases;

/**
 * Test function for reaction wheel allocation algorithms
 */
void test_reaction_wheel_allocation()
{
    LOG_INFO("Testing Reaction Wheel Allocation...");
    // Initialize test vectors separately
    float4 q_current = {4.0f, 3.0f, 2.0f, 1.0f};
    float4 q_desired = {1.0f, 2.0f, 3.0f, 4.0f};
    float3 w_current = {1.0f, 2.0f, 3.0f};
    float3 w_desired = {3.0f, 2.0f, 1.0f};

    // Log current state
    LOG_INFO("Current state: %f, %f, %f, %f, %f, %f, %f", q_current[0],
             q_current[1], q_current[2], q_current[3], w_current[0],
             w_current[1], w_current[2]);

    // Log desired state
    LOG_INFO("Desired state: %f, %f, %f, %f, %f, %f, %f", q_desired[0],
             q_desired[1], q_desired[2], q_desired[3], w_desired[0],
             w_desired[1], w_desired[2]);

    LOG_INFO("--------------------------------");

    LOG_INFO("Testing Compute Control Torque...");

    // Test the function
    float3 torque = compute_control_torque(q_current, q_desired, w_current,
                                         w_desired);
    LOG_INFO("Control torque: %f, %f, %f", torque[0], torque[1], torque[2]);

    // Test allocate_reaction_wheels
    float4 wheel_speeds = allocate_reaction_wheels(q_current, q_desired,
                                                 w_current, w_desired);
    LOG_INFO("Wheel speeds: %f, %f, %f, %f", wheel_speeds[0], wheel_speeds[1],
             wheel_speeds[2], wheel_speeds[3]);
    LOG_INFO("--------------------------------");
}