/**
 * @author Sidharth Anantha
 * @date 2025-02-22
 */

#include "desaturation.h"
#include "constants.h"
#include "linalg.h"
#include "macros.h"
#include "pico/stdlib.h"

#include <cmath>

bool saturation_detection(linalg::vec<float, 3> &reaction_wheel_1,
                          linalg::vec<float, 3> &reaction_wheel_2,
                          linalg::vec<float, 3> &reaction_wheel_3,
                          linalg::vec<float, 3> &reaction_wheel_4)
{
    /*
    Inputs: reaction wheel torques
    Outputs: boolean indicating if the reaction wheels are saturated
    Tasks: Check if the reaction wheels are saturated
    */

    // Check each reaction wheel and check if it is saturated
    if (linalg::length(reaction_wheel_1) >
        REACTION_WHEEL_SATURATION_UPPER_LIMIT *
            MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }
    if (linalg::length(reaction_wheel_2) >
        REACTION_WHEEL_SATURATION_UPPER_LIMIT *
            MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }
    if (linalg::length(reaction_wheel_3) >
        REACTION_WHEEL_SATURATION_UPPER_LIMIT *
            MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }
    if (linalg::length(reaction_wheel_4) >
        REACTION_WHEEL_SATURATION_UPPER_LIMIT *
            MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }

    // If none of the reaction wheels are saturated, return false
    return false;
}

bool desaturation_detection(linalg::vec<float, 3> &reaction_wheel_1,
                            linalg::vec<float, 3> &reaction_wheel_2,
                            linalg::vec<float, 3> &reaction_wheel_3,
                            linalg::vec<float, 3> &reaction_wheel_4)
{
    /*
    Inputs: Reaction wheel torques
    Outputs: Boolean indicating if the reaction wheels are desaturated
    Tasks: Check if the reaction wheels are desaturated yer
    */

    // Check each reaction wheel and check if it is desaturated
    if (linalg::length(reaction_wheel_1) <
        REACTION_WHEEL_SATURATION_LOWER_LIMIT *
            MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }
    if (linalg::length(reaction_wheel_2) <
        REACTION_WHEEL_SATURATION_LOWER_LIMIT *
            MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }
    if (linalg::length(reaction_wheel_3) <
        REACTION_WHEEL_SATURATION_LOWER_LIMIT *
            MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }
    if (linalg::length(reaction_wheel_4) <
        REACTION_WHEEL_SATURATION_LOWER_LIMIT *
            MAX_REACTION_WHEEL_ANGULAR_MOMENTUM)
    {
        return true;
    }

    // If none of the reaction wheels are desaturated, return false
    return false;
}

linalg::vec<float, 3> compute_total_reaction_wheel_angular_momentum(
    linalg::vec<float, 3> &reaction_wheel_1,
    linalg::vec<float, 3> &reaction_wheel_2,
    linalg::vec<float, 3> &reaction_wheel_3,
    linalg::vec<float, 3> &reaction_wheel_4)
{
    /*
    Inputs: Reaction wheel torques
    Outputs: Computed torques
    Tasks: Do a vector sum of each reaction wheel angular momentum to get total
    angular momentum
    */

    linalg::vec<float, 3> total_reaction_wheel_angular_momentum;

    // Add the torques
    for (float i = 0; i < 4; i++)
    {
        total_reaction_wheel_angular_momentum[i] =
            reaction_wheel_1[i] + reaction_wheel_2[i] + reaction_wheel_3[i] +
            reaction_wheel_4[i];
    }

    // Return the total angular momentum
    return total_reaction_wheel_angular_momentum;
}

linalg::vec<float, 3> calculate_magnetometer_dipole(
    slate_t *slate,
    linalg::vec<float, 3> &total_reaction_wheel_angular_momentum)
{
    /*
    Inputs: Slate object, magnetic dipole moment vector
    Outputs: Magnetic field torque
    Tasks: Calculate the desired magnetic dipole moment vector
        This is the desired TOTAL dipole we want from the magnetometers
    */

    // Calculate the desaturation torque
    // based on the gains we know
    // Via this gain, we convert from angular momentum to torque (* 1/s)
    linalg::vec<float, 3> desaturation_torque =
        -DESATURATION_KP * total_reaction_wheel_angular_momentum; // [N*m]

    // Calculate the magnetic dipole moment vector
    linalg::vec<float, 3> magnetic_dipole_moment_vector =
        linalg::cross(slate->b_field_local, desaturation_torque) /
        (linalg::length(slate->b_field_local) *
         linalg::length(slate->b_field_local));

    // Return the magnetic dipole moment vector
    return magnetic_dipole_moment_vector;
}
