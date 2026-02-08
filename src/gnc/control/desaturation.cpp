/**
 * @author Sidharth Anantha
 * @date 2025-02-22
 */

#include "desaturation.h"
#include "params.h"
#include "linalg.h"
#include "macros.h"
#include "pico/stdlib.h"

#include <cmath>

// TODO: This file is a very rough draft and must be refactored
// before other files are connected to it

/**
 * Check if any reaction wheel is saturated
 *
 * @param reaction_wheel_1 Angular momentum of reaction wheel 1
 * @param reaction_wheel_2 Angular momentum of reaction wheel 2
 * @param reaction_wheel_3 Angular momentum of reaction wheel 3
 * @param reaction_wheel_4 Angular momentum of reaction wheel 4
 * @return true if any wheel is saturated, false otherwise
 */
bool saturation_detection(float3 &reaction_wheel_1, float3 &reaction_wheel_2,
                          float3 &reaction_wheel_3, float3 &reaction_wheel_4)
{
    // Check each reaction wheel and check if it is saturated
    if (length(reaction_wheel_1) > REACTION_WHEEL_SATURATION_UPPER_LIMIT *
                                       MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }
    if (length(reaction_wheel_2) > REACTION_WHEEL_SATURATION_UPPER_LIMIT *
                                       MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }
    if (length(reaction_wheel_3) > REACTION_WHEEL_SATURATION_UPPER_LIMIT *
                                       MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }
    if (length(reaction_wheel_4) > REACTION_WHEEL_SATURATION_UPPER_LIMIT *
                                       MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }

    // If none of the reaction wheels are saturated, return false
    return false;
}

/**
 * Check if any reaction wheel is desaturated
 *
 * @param reaction_wheel_1 Angular momentum of reaction wheel 1
 * @param reaction_wheel_2 Angular momentum of reaction wheel 2
 * @param reaction_wheel_3 Angular momentum of reaction wheel 3
 * @param reaction_wheel_4 Angular momentum of reaction wheel 4
 * @return true if any wheel is desaturated, false otherwise
 */
bool desaturation_detection(float3 &reaction_wheel_1, float3 &reaction_wheel_2,
                            float3 &reaction_wheel_3, float3 &reaction_wheel_4)
{
    // Check each reaction wheel and check if it is desaturated
    if (length(reaction_wheel_1) < REACTION_WHEEL_SATURATION_LOWER_LIMIT *
                                       MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }
    if (length(reaction_wheel_2) < REACTION_WHEEL_SATURATION_LOWER_LIMIT *
                                       MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }
    if (length(reaction_wheel_3) < REACTION_WHEEL_SATURATION_LOWER_LIMIT *
                                       MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }
    if (length(reaction_wheel_4) < REACTION_WHEEL_SATURATION_LOWER_LIMIT *
                                       MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }

    // If none of the reaction wheels are desaturated, return false
    return false;
}

/**
 * Compute total angular momentum from all reaction wheels
 *
 * @param reaction_wheel_1 Angular momentum of reaction wheel 1
 * @param reaction_wheel_2 Angular momentum of reaction wheel 2
 * @param reaction_wheel_3 Angular momentum of reaction wheel 3
 * @param reaction_wheel_4 Angular momentum of reaction wheel 4
 * @return Total angular momentum vector
 */
float3 compute_total_reaction_wheel_angular_momentum(float3 &reaction_wheel_1,
                                                     float3 &reaction_wheel_2,
                                                     float3 &reaction_wheel_3,
                                                     float3 &reaction_wheel_4)
{
    float3 total_reaction_wheel_angular_momentum;

    // Add the angular momentum components
    for (int i = 0; i < 3; i++)
    {
        total_reaction_wheel_angular_momentum[i] =
            reaction_wheel_1[i] + reaction_wheel_2[i] + reaction_wheel_3[i] +
            reaction_wheel_4[i];
    }

    // Return the total angular momentum
    return total_reaction_wheel_angular_momentum;
}

/**
 * Calculate desired magnetic dipole moment for reaction wheel desaturation
 *
 * @param slate Satellite state containing magnetic field data
 * @param total_reaction_wheel_angular_momentum Total angular momentum of all
 * wheels
 * @return Desired magnetic dipole moment vector
 */
float3
calculate_magnetometer_dipole(slate_t *slate,
                              float3 &total_reaction_wheel_angular_momentum)
{
    // Calculate the desaturation torque
    // based on the gains we know
    // Via this gain, we convert from angular momentum to torque (* 1/s)
    float3 desaturation_torque =
        -DESATURATION_KP * total_reaction_wheel_angular_momentum; // [N*m]

    // Calculate the magnetic dipole moment vector
    float3 magnetic_dipole_moment_vector =
        cross(slate->b_body, desaturation_torque) /
        (length(slate->b_body) * length(slate->b_body));

    // Return the magnetic dipole moment vector
    return magnetic_dipole_moment_vector;
}
