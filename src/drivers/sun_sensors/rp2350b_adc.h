/**
 * @author Lundeen Cahilly
 * @date 2025-08-20
 *
 * This file reads data from the YZ (+-) photodiodes
 * using the ADC pins on a RP2350b chip. These support
 * 12-bit ADC resolution and a reference voltage of 3.3V.
 */

#pragma once

#include "linalg.h"
#include "macros.h"
using namespace linalg::aliases;

bool rp2350b_adc_init(void);

bool rp2350b_adc_get_reading(uint8_t channel, uint16_t *value);

bool rp2350b_adc_get_voltage(uint8_t channel, float *voltage);

bool rp2350b_adc_read_all_channels(uint16_t values_out[8]);

bool rp2350b_adc_read_all_voltages(float voltages_out[8]);