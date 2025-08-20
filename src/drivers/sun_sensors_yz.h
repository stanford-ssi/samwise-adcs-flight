/**
 * @author Lundeen Cahilly
 * @date 2025-08-20
 *
 * This file reads data from the YZ (+-) sun sensors
 * using the ADC pins on a RP2350b chip. These support
 * 12-bit ADC resolution and a reference voltage of 3.3V.
 */

#pragma once

#include "linalg.h"
using namespace linalg::aliases;

/**
 * Initialize the YZ sun sensors by setting up the ADC hardware
 * @return true if initialization successful, false otherwise
 */
bool sun_sensors_yz_init(void);

/**
 * Get a raw ADC reading from a specific sun sensor channel
 * @param channel ADC channel number (0-7) [GPIO 40-47]
 * @param value Pointer to store the 12-bit ADC value (0-4095)
 * @return true if reading successful, false otherwise
 */
bool sun_sensors_yz_get_reading(uint8_t channel, uint8_t *value);

/**
 * Get a voltage reading from a specific sun sensor channel
 * @param channel ADC channel number (0-3 for Y+, Y-, Z+, Z-)
 * @param voltage Pointer to store the calculated voltage
 * @return true if reading successful, false otherwise
 */
bool sun_sensors_yz_get_voltage(uint8_t channel, float *voltage);

/**
 * Read all 8 sun sensor channels and return raw ADC values
 * @param values_out Array to store 8-bit ADC values for all channels
 * @return true if all readings successful, false otherwise
 */
bool sun_sensors_yz_read_all_channels(uint8_t values_out[8]);

/**
 * Read all 8 sun sensor channels and return voltage values
 * @param voltages_out Array to store voltage values for all channels
 * @return true if all readings successful, false otherwise
 */
bool sun_sensors_yz_read_all_voltages(float voltages_out[8]);