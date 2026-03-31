/**
 * @author Lundeen Cahilly and Sage Wu
 * @date 2025-06-03
 *
 * This file contains functions for reading ADC data from the ADS7830
 * using I2C on a RP2350 chip for sun pyramids voltage measurements
 */
#pragma once

#include "linalg.h"
#include "macros.h"
using namespace linalg::aliases;

// ADS7830 ADC Configuration (ADCS board v1.8)
// Reads Y/Z photodiode sensors via I2C1
constexpr float VREF_ADS7830 = 2.5f; // Internal reference voltage
constexpr uint16_t BIT_RESOLUTION_ADS7830 = 8;
constexpr uint16_t MAX_VALUE_ADS7830 =
    (1 << BIT_RESOLUTION_ADS7830); // 8-bit ADC max value

bool ads7830_init(void);

bool ads7830_get_voltage(uint8_t channel, float *voltage, float vref);

bool ads7830_read_all_channels(uint8_t values_out[8]);

bool ads7830_read_all_voltages(float voltages_out[8]);
