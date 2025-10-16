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

bool ads7830_init(void);

bool ads7830_get_voltage(uint8_t channel, float *voltage, float vref);

bool ads7830_read_all_channels(uint8_t values_out[8]);

bool ads7830_read_all_voltages(float voltages_out[8]);