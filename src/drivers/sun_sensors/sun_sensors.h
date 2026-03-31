#pragma once

#include "linalg.h"
#include "pico/stdlib.h"
#include "constants.h"

#include "drivers/sun_sensors/ads7830.h"
#include "drivers/sun_sensors/rp2350b_adc.h"

using namespace linalg::aliases;

// CONSTANT DECLARATIONS
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

// Scales sun sensor readings from different ADCs to match
constexpr uint16_t SUN_SENSOR_SATURATION_VALUE = static_cast<uint16_t>(
    VREF_ADS7830 / VREF_RP2350B_ADC * MAX_VALUE_RP2350B_ADC);

static constexpr uint16_t SUN_SENSOR_ACTIVE_THRESHOLD =
    500; // TODO: determine what makes sense for our system in LEO (0.5 * max
         // intensity think?) given the Earth's reflected light. this will limit
         // our effective FOV for each sensor

typedef struct sun_sensor_data {
    bool sun_sensor_alive[16];
    bool data_valid[16];

    uint16_t intensities[16];
    float voltages[16];

    bool sun_vector_valid;
    float3 sun_vector_body;
} sun_sensor_data_t;
