/**
 * @author Sage Wu
 * @date 2025-06-03
 *
 * This file contains functions for reading ADC data from the ADS7830
 * using I2C on a RP2350 chip for sun sensor voltage measurements
 */
#pragma once

#include "linalg.h"
using namespace linalg::aliases;

/**
 * Initialize ADS7830 ADC
 *
 * This function initializes the I2C interface and verifies ADC communication
 * by performing a test conversion on channel 0.
 *
 * @return bool True on success, false on failure
 */
bool ads7830_init(void);

/**
 * Get ADC reading from specified channel
 *
 * This function reads the ADC value from the specified channel and
 * returns the raw 8-bit value (0-255) representing the input voltage.
 *
 * @param channel ADC channel (0-7)
 * @param value Pointer to store the 8-bit ADC reading
 * @return bool True on success, false on failure
 */
bool ads7830_get_reading(uint8_t channel, uint8_t *value);

/**
 * Get voltage reading from specified channel
 *
 * This function reads the ADC value and converts it to voltage using
 * the specified reference voltage (typically 3.3V for your system).
 *
 * @param channel ADC channel (0-7)
 * @param voltage Pointer to store voltage in volts
 * @param vref Reference voltage in volts
 * @return bool True on success, false on failure
 */
bool ads7830_get_voltage(uint8_t channel, float *voltage, float vref);