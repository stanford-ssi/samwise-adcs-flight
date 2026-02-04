/**
 * @author The ADCS team
 * @date 2025-02-10
 * Definition of all constants used in GNC
 */

#include "linalg.h"
using namespace linalg::aliases;

#pragma once

// ========================================================================
//          GENERAL SPECIFICATIONS
// ========================================================================

constexpr uint32_t NUM_SUN_SENSORS = 16; // 8 pyramid, 8 yz (+-)
constexpr uint32_t NUM_REACTION_WHEELS = 4;

// IMU Calibration - zero rotation reading in radians per second
constexpr float3 IMU_ZERO_READING_RPS = {0.0f, 0.0f, 0.0f};

// ========================================================================
//          GPS SPECIFICATIONS
// ========================================================================
constexpr uint32_t GPS_DATA_EXPIRATION_MS =
    15000; // At 7.6 km/s, gives us 114 km max error

// ========================================================================
//          MAGNETOMETER CALIBRATION
// ========================================================================

// TODO: Update with FLIGHT model (see scripts/calibrations/magnetometer)

// Hard iron offset correction (sensor units)
constexpr float3 MAG_HARD_IRON_OFFSET =
    float3{-2.540693, 16.138800, -24.242187};

// Soft iron matrix correction (in sensor units)
constexpr float3x3 MAG_SOFT_IRON_MATRIX = {{0.026047f, 0.000375f, -0.001275f},
                                           {0.000375f, 0.027098f, -0.000445f},
                                           {-0.001275f, -0.000445f, 0.026726f}};

// ========================================================================
//          MAGNETOMETER SAMPLING
// ========================================================================

// Time to turn off the magnetorquers so we can measure the magnetometer
// TODO: test on FLIGHT model
constexpr uint32_t MAGNETOMETER_FIELD_SETTLE_TIME_MS = 20; // [ms]

// ========================================================================
//          WORLD CONSTANTS
// ========================================================================

constexpr float R_E = 6378.0f; // Earth equatorial radius in km
constexpr float MU_EARTH =
    398600.4418f; // Earth gravitational parameter in km^3/s^2

// (These are generally useful)
constexpr float DEG_TO_RAD = 0.01745329251;
constexpr float RAD_TO_DEG = 57.2957795131;
constexpr float SQRT_2_INV = 0.7071067811865476f; // 1 / sqrt(2)

// ========================================================================
//          MATH CONSTANTS
// ========================================================================

constexpr float3x3 identity3x3 = {
    {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

constexpr float identity6x6[6 * 6] = {
    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
};

// ========================================================================
//          STATE TRANSITION THRESHOLDS
// ========================================================================

// Emergency power saving thresholds
constexpr float BATTERY_VOLTAGE_SAFE = 7.0f; // [V]

// Rotation thresholds for state transitions - TODO: pick good values!
constexpr float W_ENTER_SAFE_THRESHOLD = (100.0 * DEG_TO_RAD); // in rad/s
constexpr float W_EXIT_SAFE_THRESHOLD = (90.0 * DEG_TO_RAD);   // in rad/s

constexpr float W_ENTER_DETUMBLE_THRESHOLD = (10.0 * DEG_TO_RAD); // in rad/s
constexpr float W_EXIT_DETUMBLE_THRESHOLD = (1.0 * DEG_TO_RAD);   // in rad/s

// ========================================================================
//          REACTION WHEEL SPECS
// ========================================================================

// Spec sheet:
// https://www.faulhaber.com/fileadmin/Import/Media/EN_2610_B_DFF.pdf
// Reaction wheel MOI: This is the moment of inertia of the reaction wheel about
// its axis of rotation. Same for all reaction wheels, irregardless of oriented
// axis
constexpr float REACTION_WHEEL_MOI = 7.90e-7; // [kg*m^2]

// Reaction wheel max angular momentum:
constexpr float MAX_REACTION_WHEEL_ANGULAR_MOMENTUM =
    (40000 / 60) * REACTION_WHEEL_MOI; // [kg*m^2/s]

// Reaction wheel upper proportional limit:
constexpr float REACTION_WHEEL_SATURATION_UPPER_LIMIT = 0.9;

// Reaction wheel lower proportional limit:
constexpr float REACTION_WHEEL_SATURATION_LOWER_LIMIT = 0.1;

// ========================================================================
//          SATELLITE INERTIA
// ========================================================================

constexpr float3 I_PRINCIPAL = {
    0.0237644, 0.0155789,
    0.0131767}; // Satellite inertia tensor in principal axes [kg m^2]

constexpr float3x3 PRINCIPAL_AXES_DCM = {
    {-0.716356336507477, 0.004404837035993194, -0.6977207152982292},
    {-0.6975131547302291, -0.029713909797595195, 0.7159556428598237},
    {-0.017578342466487904, 0.999548738669219,
     0.02435817934296709}}; // DCM from body to principal axes

constexpr quaternion Q_BODY_TO_PRINCIPAL = {
    0.268793, -0.644648, -0.665287,
    0.263765}; // Quaternion from body to principal axes (scalar-last)

constexpr float3x3 I_BODY = {
    {0.01861, 0.00529, 0.0001439},
    {0.00529, 0.01833, 0.0000584709},
    {0.0001439, 0.0000584709,
     0.01558}}; // Satellite inertia tensor in body frame [kg m^2]

// ========================================================================
//          SENSOR NOISE
// ========================================================================
// See IMU datasheet:
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf
// Choose 0.5 º/s as a rough approximation of large noise sources (bias +
// temperature)
constexpr float GYRO_STD_DEV = 0.0087266; // [rad/s]

// TODO: Pick good values
constexpr float SUN_SENSOR_STD = 0.01;
constexpr float MAG_SENSOR_STD = 0.01;

// ========================================================================
//          DESATURATION GAINS
// ========================================================================

// Desaturation gains for each reaction wheel
constexpr float DESATURATION_KP = 0.01; // [1/s]

// ========================================================================
//          POWER MONITORING
// ========================================================================

constexpr float ADCS_POWER_SENSE_RESISTOR = 0.0207f; // [ohms]

// ========================================================================
//          SUN SENSOR NORMALS
// ========================================================================

// Define all sun sensor normal vectors (NUM_SUN_SENSORS x 3 matrix)
// TODO: swap for new adcs board
const float SUN_SENSOR_NORMALS[NUM_SUN_SENSORS][3] = {
    // Pyramid group 1 on +X face (0-3)
    {SQRT_2_INV, 0, SQRT_2_INV},  // sun_pyramid_1_1
    {SQRT_2_INV, SQRT_2_INV, 0},  // sun_pyramid_1_2
    {SQRT_2_INV, 0, -SQRT_2_INV}, // sun_pyramid_1_3
    {SQRT_2_INV, -SQRT_2_INV, 0}, // sun_pyramid_1_4
    // Pyramid group 2 on -X face (4-7)
    {-SQRT_2_INV, 0, SQRT_2_INV},  // sun_pyramid_2_1
    {-SQRT_2_INV, -SQRT_2_INV, 0}, // sun_pyramid_2_2
    {-SQRT_2_INV, 0, -SQRT_2_INV}, // sun_pyramid_2_3
    {-SQRT_2_INV, SQRT_2_INV, 0},  // sun_pyramid_2_4
    // Y+ sensors (8-9)
    {0, -1, 0}, // y+ sensor 1
    {0, -1, 0}, // y+ sensor 2
    // Y- sensors (10-11)
    {0, 1, 0}, // y- sensor 1
    {0, 1, 0}, // y- sensor 2
    // Z+ face sensors (12-13)
    {0, 0, 1}, // z+ sensor 1
    {0, 0, 1}, // z+ sensor 2
    // Z- face sensors (14-15)
    {0, 0, -1}, // z- sensor 1
    {0, 0, -1}, // z- sensor 2
};

// ========================================================================
//          SUN SENSOR ADC NORMALIZATION
// ========================================================================

// RP2350B ADC Configuration (ADCS board v1.8)
// Reads sun pyramid sensors (GPIO 40-47)
constexpr float VREF_RP2350B_ADC = 3.3f;
constexpr uint16_t BIT_RESOLUTION_RP2350B_ADC = 12;
constexpr uint16_t MAX_VALUE_RP2350B_ADC =
    (1 << BIT_RESOLUTION_RP2350B_ADC); // 4095 for 12-bit ADC

// ADS7830 ADC Configuration (ADCS board v1.8)
// Reads Y/Z photodiode sensors via I2C1
constexpr float VREF_ADS7830 = 2.5f; // Internal reference voltage
constexpr uint16_t BIT_RESOLUTION_ADS7830 = 8;
constexpr uint16_t MAX_VALUE_ADS7830 =
    (1 << BIT_RESOLUTION_ADS7830); // 8-bit ADC max value

// Scales sun sensor readings from different ADCs to match
constexpr uint16_t SUN_SENSOR_CLIP_VALUE = static_cast<uint16_t>(
    VREF_ADS7830 / VREF_RP2350B_ADC * MAX_VALUE_RP2350B_ADC);
