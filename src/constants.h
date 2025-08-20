/**
 * @author The ADCS team
 * @date 2025-02-10
 * Definition of all constants used in GNC
 */

#include "linalg.h"
using namespace linalg::aliases;

#pragma once

// General specifications
constexpr uint32_t NUM_SUN_SENSORS = 16; // 8 pyramid, 8 yz (+-)
constexpr uint32_t NUM_REACTION_WHEELS = 4;

// IMU Calibration - zero rotation reading in radians per second
constexpr float3 IMU_ZERO_READING_RPS = {0.0f, 0.0f, 0.0f};

// (These are generally useful)
constexpr float DEG_TO_RAD = 0.01745329251;
constexpr float RAD_TO_DEG = 57.2957795131;

// Rotation thresholds for state transitions - TODO: pick good values!
constexpr float W_COOL_DOWN_ENTER_THRESHOLD = (100.0 * DEG_TO_RAD); // in rad/s
constexpr float W_COOL_DOWN_EXIT_THRESHOLD = (90.0 * DEG_TO_RAD);   // in rad/s

constexpr float W_ENTER_DETUMBLE_THRESHOLD = (10.0 * DEG_TO_RAD); // in rad/s
constexpr float W_EXIT_DETUMBLE_THRESHOLD = (1.0 * DEG_TO_RAD);   // in rad/s

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

// #### SATELLITE INERTIA ###
// !!!!!!!!!!!!!!!! TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!
// These are calculated from a really hacky estimate
// made back in fall quarter. We should replace these with
// something much more accurate, ideally measured with the actual
// flight model before launch
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
constexpr float3 SATELLITE_INERTIA = {0.01461922201, 0.0412768466,
                                      0.03235309961}; // Principle axes [kg m^2]

// TODO: Add principal axes to body rotation

// #### DESATURATION GAINS ####
// Desaturation gains for each reaction wheel
constexpr float DESATURATION_KP = 0.01; // [1/s]

// #### ADM1176 POWER MONITORING ####
constexpr float ADCS_POWER_SENSE_RESISTOR = 0.0207f; // [ohms]

// #### SUN SENSOR NORMALIZATION ####
constexpr float VREF_ADS7830 = 2.5f;
constexpr float VREF_RP2350B_ADC = 3.3f;
constexpr uint16_t MAX_VALUE_RP2350B_ADC = 4095; // 12-bit ADC max value
constexpr uint16_t MAX_VALUE_ADS7830 = 255;      // 8-bit ADC max value
constexpr uint16_t SUN_SENSOR_CLIP_VALUE = static_cast<uint16_t>(
    VREF_ADS7830 / VREF_RP2350B_ADC * MAX_VALUE_RP2350B_ADC);