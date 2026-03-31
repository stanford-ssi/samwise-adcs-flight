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

// RP2350B ADC Configuration (ADCS board v1.8)
// Reads sun pyramid sensors (GPIO 40-47)
constexpr float VREF_RP2350B_ADC = 3.3f;
constexpr uint16_t BIT_RESOLUTION_RP2350B_ADC = 12;
constexpr uint16_t MAX_VALUE_RP2350B_ADC =
    (1 << BIT_RESOLUTION_RP2350B_ADC); // 4095 for 12-bit ADC

bool rp2350b_adc_init(void);

bool rp2350b_adc_get_reading(uint8_t channel, uint16_t *value);

bool rp2350b_adc_get_voltage(uint8_t channel, float *voltage);

bool rp2350b_adc_read_all_channels(uint16_t values_out[8]);

bool rp2350b_adc_read_all_voltages(float voltages_out[8]);
