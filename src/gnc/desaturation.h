#pragma once

#include "constants.h"
#include "linalg.h"
#include "slate.h"

/**
 * Detects if any reaction wheel is saturated
 * @param reaction_wheel_1 Angular momentum of reaction wheel 1
 * @param reaction_wheel_2 Angular momentum of reaction wheel 2
 * @param reaction_wheel_3 Angular momentum of reaction wheel 3
 * @param reaction_wheel_4 Angular momentum of reaction wheel 4
 * @return True if any reaction wheel is saturated, false otherwise
 */
bool saturation_detection(linalg::vec<float, 3> &reaction_wheel_1,
                          linalg::vec<float, 3> &reaction_wheel_2,
                          linalg::vec<float, 3> &reaction_wheel_3,
                          linalg::vec<float, 3> &reaction_wheel_4);

/**
 * Detects if any reaction wheel is desaturated
 * @param reaction_wheel_1 Angular momentum of reaction wheel 1
 * @param reaction_wheel_2 Angular momentum of reaction wheel 2
 * @param reaction_wheel_3 Angular momentum of reaction wheel 3
 * @param reaction_wheel_4 Angular momentum of reaction wheel 4
 * @return True if any reaction wheel is desaturated, false otherwise
 */
bool desaturation_detection(linalg::vec<float, 3> &reaction_wheel_1,
                            linalg::vec<float, 3> &reaction_wheel_2,
                            linalg::vec<float, 3> &reaction_wheel_3,
                            linalg::vec<float, 3> &reaction_wheel_4);

/**
 * Computes the total angular momentum of all reaction wheels
 * @param reaction_wheel_1 Angular momentum of reaction wheel 1
 * @param reaction_wheel_2 Angular momentum of reaction wheel 2
 * @param reaction_wheel_3 Angular momentum of reaction wheel 3
 * @param reaction_wheel_4 Angular momentum of reaction wheel 4
 * @return Total angular momentum vector
 */
linalg::vec<float, 3> compute_total_reaction_wheel_angular_momentum(
    linalg::vec<float, 3> &reaction_wheel_1,
    linalg::vec<float, 3> &reaction_wheel_2,
    linalg::vec<float, 3> &reaction_wheel_3,
    linalg::vec<float, 3> &reaction_wheel_4);

/**
 * Calculates the required magnetic dipole moment for desaturation
 * @param slate Pointer to the slate object containing system state
 * @param total_reaction_wheel_angular_momentum Total angular momentum of all
 * reaction wheels
 * @return Required magnetic dipole moment vector
 */
linalg::vec<float, 3> calculate_magnetometer_dipole(
    slate_t *slate,
    linalg::vec<float, 3> &total_reaction_wheel_angular_momentum);