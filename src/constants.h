/**
 * @author The ADCS team
 * @date 2025-02-10
 * Definition of all constants used in GNC
 */

#pragma once

// #### REACTION WHEEL SPECS ####
// Spec sheet:
// https://www.faulhaber.com/fileadmin/Import/Media/EN_2610_B_DFF.pdf Reaction
// wheel MOI: This is the moment of inertia of the reaction wheel about its axis
// of rotation Same for all reaction wheels, irregardless of oriented axis
constexpr float REACTION_WHEEL_MOI = 7.90e-7; // [kg*m^2]
// Reaction wheel max angular momentum:
constexpr float MAX_REACTION_WHEEL_ANGULAR_MOMENTUM =
    (40000 / 60) * REACTION_WHEEL_MOI; // [kg*m^2/s]
// Reaction wheel upper proportional limit:
constexpr float REACTION_WHEEL_SATURATION_UPPER_LIMIT = 0.9;
// Reaction wheel lower proportional limit:
constexpr float REACTION_WHEEL_SATURATION_LOWER_LIMIT = 0.1;

// #### DESATURATION GAINS ####
// Desaturation gains for each reaction wheel
constexpr float DESATURATION_KP = 0.01; // [1/s]
