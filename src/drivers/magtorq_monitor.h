/**
 * @author Lundeen Cahilly
 * @date 2025-08-10
 *
 * This file contains functions for reading ADC data from the ADS7830
 * using I2C on a RP2350 chip for magnetorquer current measurements
 */
#pragma once

#include "linalg.h"
using namespace linalg::aliases;

/**
 * Initialize ADS7830 ADC for magnetorquer current monitoring
 *
 * This function initializes the I2C interface and verifies ADC communication
 * by performing a test conversion on channel 0. Configures the ADC for
 * current sensing applications.
 *
 * @return bool True on success, false on failure
 */
bool magtorq_monitor_init(void);

/**
 * Get ADC reading from specified magnetorquer channel
 *
 * This function reads the ADC value from the specified channel and
 * returns the raw 8-bit value (0-255) representing the current sense voltage.
 *
 * @param channel ADC channel (0-7) corresponding to magnetorquer axis
 * @param value Pointer to store the 8-bit ADC reading
 * @return bool True on success, false on failure
 */
bool magtorq_get_reading(uint8_t channel, uint8_t *value);

/**
 * Get current reading from specified magnetorquer channel
 *
 * This function reads the ADC value and converts it to current using
 * the current sense resistor value and amplifier gain.
 *
 * @param channel ADC channel (0-7) corresponding to magnetorquer axis
 * @param current Pointer to store current in amperes
 * @param sense_resistor Current sense resistor value in ohms
 * @param amp_gain Current sense amplifier gain (V/V)
 * @param vref Reference voltage in volts
 * @return bool True on success, false on failure
 */
bool magtorq_get_current(uint8_t channel, float *current, float sense_resistor,
                         float amp_gain, float vref);

/**
 * Read all magnetorquer current channels
 *
 * This function reads all 8 ADC channels and returns the raw 8-bit values
 * for magnetorquer current monitoring.
 *
 * @param values_out Array to store 8 ADC readings (0-255)
 * @return bool True if all channels read successfully, false otherwise
 */
bool magtorq_read_all_channels(uint8_t values_out[8]);

/**
 * Read all magnetorquer currents
 *
 * This function reads all 8 ADC channels and converts them to current values
 * using the specified current sensing parameters.
 *
 * @param currents_out Array to store 8 current readings (in amperes)
 * @param sense_resistor Current sense resistor value in ohms
 * @param amp_gain Current sense amplifier gain (V/V)
 * @param vref Reference voltage in volts
 * @return bool True if all channels read successfully, false otherwise
 */
bool magtorq_read_all_currents(float currents_out[8], float sense_resistor,
                               float amp_gain, float vref);

/**
 * Get magnetorquer current vector (X, Y, Z axes)
 *
 * This function reads the current from the first 3 channels (assumed to be
 * X, Y, Z magnetorquer axes) and returns them as a 3D vector.
 *
 * @param current_vector 3D vector to store current readings [X, Y, Z] in
 * amperes
 * @param sense_resistor Current sense resistor value in ohms
 * @param amp_gain Current sense amplifier gain (V/V)
 * @param vref Reference voltage in volts
 * @return bool True if all axes read successfully, false otherwise
 */
bool magtorq_get_current_vector(float3 &current_vector, float sense_resistor,
                                float amp_gain, float vref);