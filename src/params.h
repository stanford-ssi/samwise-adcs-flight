/*
 * @author Lundeen Cahilly
 * @date 2026-02-07
 * Definition of all satellite parameters
 */

#pragma once

#include <cstdint>

#include "constants.h"
#include "linalg.h"

using namespace linalg::aliases;

// ========================================================================
//          INERTIA
// ========================================================================
// TODO: update for FLIGHT model
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
//          STATE TRANSITION
// ========================================================================
// Rotation thresholds for state transitions - TODO: pick good values!
constexpr float W_ENTER_SAFE_THRESHOLD = (100.0 * DEG_TO_RAD); // in rad/s
constexpr float W_EXIT_SAFE_THRESHOLD = (90.0 * DEG_TO_RAD);   // in rad/s

constexpr float W_ENTER_DETUMBLE_THRESHOLD = (10.0 * DEG_TO_RAD); // in rad/s
constexpr float W_EXIT_DETUMBLE_THRESHOLD = (1.0 * DEG_TO_RAD);   // in rad/s

// ========================================================================
//          CALIBRATIONS
// ========================================================================
// TODO: Update with FLIGHT model (see scripts/calibrations/magnetometer)
// Hard iron offset correction (sensor units)
constexpr float3 MAG_HARD_IRON_OFFSET =
    float3{-2.540693, 16.138800, -24.242187};

// Soft iron matrix correction (in sensor units)
constexpr float3x3 MAG_SOFT_IRON_MATRIX = {{0.026047f, 0.000375f, -0.001275f},
                                           {0.000375f, 0.027098f, -0.000445f},
                                           {-0.001275f, -0.000445f, 0.026726f}};

// IMU Calibration - zero rotation reading in radians per second
constexpr float3 IMU_ZERO_READING_RPS = {0.0f, 0.0f, 0.0f};

// ========================================================================
//          MEASUREMENT THRESHOLDS
// ========================================================================
static constexpr uint16_t SUN_SENSOR_ACTIVE_THRESHOLD =
    500; // TODO: determine what makes sense for our system in LEO (0.5 * max
         // intensity think?) given the Earth's reflected light. this will limit
         // our effective FOV for each sensor

constexpr uint32_t GPS_DATA_EXPIRATION_MS =
    15000; // At 7.6 km/s, gives us 114 km max error

// Time to turn off the magnetorquers so we can measure the magnetometer
// TODO: test on FLIGHT model
constexpr uint32_t MAGNETOMETER_FIELD_SETTLE_TIME_MS = 20; // [ms]

// ========================================================================
//          POWER
// =========================================================================
// Emergency power saving thresholds
constexpr float BATTERY_VOLTAGE_SAFE = 7.0f; // [V]

constexpr float ADCS_POWER_SENSE_RESISTOR = 0.0207f; // [ohms]

// ========================================================================
//          SENSOR NOISE
// ========================================================================
// TODO: use (more) reasonable values
// Datasheet:
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf
constexpr float IMU_GYRO_VARIANCE =
    1.52309e-6f; // rad^2/s^2 For BMI270 at 50hz - TODO: change the update rate,
                 // we are not doing 50hz
constexpr float IMU_DRIFT_VARIANCE =
    1.52309e-8f; // rad^2/s^4 (~100x smaller than gyro variance) - TODO: measure
                 // this!

// TODO: use reasonable values from datasheet and testing magnetometer
// datasheet: https://www.tri-m.com/products/pni/RM3100-User-Manual.pdf
constexpr float SUN_VECTOR_VARIANCE =
    (1.0f * DEG_TO_RAD) * (1.0f * DEG_TO_RAD); // Sun sensor noise ~+-2 degrees
constexpr float MAGNETOMETER_VARIANCE =
    (0.02f * DEG_TO_RAD) *
    (0.02f * DEG_TO_RAD); // Magnetometer noise ~ 2 degrees

// ========================================================================
//          SUN SENSORS
// ========================================================================
constexpr uint32_t NUM_SUN_SENSORS = 16; // 8 pyramid, 8 yz (+-)
constexpr uint32_t NUM_SUN_SENSOR_OPPOSITE_PAIRS = 8;

// Define all sun sensor normal vectors (NUM_SUN_SENSORS x 3 matrix)
constexpr float3 SUN_SENSOR_NORMALS[NUM_SUN_SENSORS] = {
    {SQRT_2_INV, 0, SQRT_2_INV},
    {SQRT_2_INV, SQRT_2_INV, 0},
    {SQRT_2_INV, 0, -SQRT_2_INV},
    {SQRT_2_INV, -SQRT_2_INV, 0},
    {-SQRT_2_INV, 0, SQRT_2_INV},
    {-SQRT_2_INV, -SQRT_2_INV, 0},
    {-SQRT_2_INV, 0, -SQRT_2_INV},
    {-SQRT_2_INV, SQRT_2_INV, 0},
    {0, -1, 0},
    {0, -1, 0},
    {0, 1, 0},
    {0, 1, 0},
    {0, 0, 1},
    {0, 0, 1},
    {0, 0, -1},
    {0, 0, -1},
};

constexpr int SUN_SENSOR_OPPOSITE_PAIRS[NUM_SUN_SENSOR_OPPOSITE_PAIRS][2] = {
    {0, 6},   {1, 5},  {2, 4}, {3, 7}, // pyramid opposites
    {8, 10},  {9, 11},                 // Y+ vs Y- pairs
    {12, 14}, {13, 15}                 // Z+ vs Z- pairs
};

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
constexpr uint16_t SUN_SENSOR_SATURATION_VALUE = static_cast<uint16_t>(
    VREF_ADS7830 / VREF_RP2350B_ADC * MAX_VALUE_RP2350B_ADC);

// ========================================================================
//          REACTION WHEELS
// ========================================================================
// Spec sheet:
// https://www.faulhaber.com/fileadmin/Import/Media/EN_2610_B_DFF.pdf
// Reaction wheel MOI: This is the moment of inertia of the reaction wheel about
// its axis of rotation. Same for all reaction wheels, irregardless of oriented
// axis
constexpr uint32_t NUM_REACTION_WHEELS = 4;
constexpr float REACTION_WHEEL_MOI = 7.90e-7; // [kg*m^2]

// Reaction wheel max angular momentum:
constexpr float MAX_REACTION_WHEEL_ANGULAR_MOMENTUM =
    (40000 / 60) * REACTION_WHEEL_MOI; // [kg*m^2/s]

// Reaction wheel upper proportional limit:
constexpr float REACTION_WHEEL_SATURATION_UPPER_LIMIT = 0.9;

// Reaction wheel lower proportional limit:
constexpr float REACTION_WHEEL_SATURATION_LOWER_LIMIT = 0.1;

// ========================================================================
//          CONTROL GAINS
// ========================================================================
// Desaturation gains for each reaction wheel
constexpr float DESATURATION_KP = 0.01; // [1/s]