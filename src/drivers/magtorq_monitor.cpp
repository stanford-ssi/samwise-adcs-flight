/**
 * @author Lundeen Cahilly
 * @date 2025-08-10
 *
 * This file defines utilities for controlling the ADS7830 ADC for
 * magnetorquer current monitoring.
 *
 * ADS7830 is an 8-bit, 8-channel ADC with I2C interface for magnetorquer
 * current measurements.
 */

#include "magtorq_monitor.h"

#include "constants.h"
#include "macros.h"
#include "pins.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#include <stdint.h>
#include <stdlib.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

/******************************************************************************/
/*!                 Macro definitions                                         */

/*! I2C Device address for magnetorquer monitor ADS7830 */
#define MAGTORQ_ADS7830_I2C_ADDR (0x48)

/*! I2C timeout in milliseconds */
#define MAGTORQ_I2C_TIMEOUT_MS (10)

/*! Maximum ADC channels */
#define MAGTORQ_MAX_CHANNELS (8)

/*! Expected 8-bit ADC resolution */
#define MAGTORQ_RESOLUTION_BITS (8)
#define MAGTORQ_MAX_VALUE (255)

/******************************************************************************/
/*!                Static variable definition                                 */

// Using internal reference (2.5V) for magnetorquer current monitoring
static const uint8_t magtorq_channel_commands[8] = {
    0x8C, 0xCC, 0x9C, 0xDC, 0xAC, 0xEC, 0xBC, 0xFC // PD1=1, PD0=1
};
#define MAGTORQ_VREF (2.5f) // Reference voltage for internal reference

/*! I2C instance for magnetorquer ADC communication (using different channel) */
static i2c_inst_t *magtorq_i2c_inst = i2c0; // Using i2c0 instead of i2c1

/******************************************************************************/
/*!                Static function definitions                                */

/**
 * Thin wrapper around writing to the I2C bus
 */
static int8_t magtorq_write_i2c(uint8_t *data, uint16_t count)
{
    if (count < 1)
    {
        return -1;
    }

    absolute_time_t timeout = make_timeout_time_ms(MAGTORQ_I2C_TIMEOUT_MS);
    int result =
        i2c_write_blocking_until(magtorq_i2c_inst, MAGTORQ_ADS7830_I2C_ADDR,
                                 data, count, false, timeout);

    if (result == PICO_ERROR_GENERIC || result == PICO_ERROR_TIMEOUT)
    {
        return -1;
    }

    return 0;
}

/**
 * Thin wrapper around reading from the I2C bus
 */
static int8_t magtorq_read_i2c(uint8_t *data, uint16_t count)
{
    if (count < 1)
    {
        return -1;
    }

    absolute_time_t timeout = make_timeout_time_ms(MAGTORQ_I2C_TIMEOUT_MS);
    int result =
        i2c_read_blocking_until(magtorq_i2c_inst, MAGTORQ_ADS7830_I2C_ADDR,
                                data, count, false, timeout);

    if (result == PICO_ERROR_GENERIC || result == PICO_ERROR_TIMEOUT)
    {
        return -1;
    }

    return 0;
}

/*!
 *  @brief Prints the execution status of magnetorquer ADC operations.
 */
static void magtorq_print_result(int8_t rslt, const char *operation)
{
    if (rslt != 0)
    {
        LOG_ERROR(
            "[magtorq-monitor] Magnetorquer ADC %s failed with result: %d",
            operation, rslt);
    }
}

/******************************************************************************/
/*!                Public function definitions                                */

/**
 * Initialize I2C pins for magnetorquer ADS7830 communication
 */
static void magtorq_init_pins()
{
    // Enable magnetorquer current sensing (if there's a specific enable pin)
    // gpio_init(SAMWISE_MAGTORQ_EN_PIN);
    // gpio_set_dir(SAMWISE_MAGTORQ_EN_PIN, GPIO_OUT);
    // gpio_put(SAMWISE_MAGTORQ_EN_PIN, 1); // Enable current sensing

    // Initialize I2C0 pins for magnetorquer monitoring
    gpio_init(SAMWISE_ADCS_I2C0_SDA);
    gpio_init(SAMWISE_ADCS_I2C0_SCL);
    gpio_set_function(SAMWISE_ADCS_I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(SAMWISE_ADCS_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SAMWISE_ADCS_I2C0_SDA);
    gpio_pull_up(SAMWISE_ADCS_I2C0_SCL);
}

/**
 * Initialize the magnetorquer monitor ADS7830 ADC
 */
bool magtorq_monitor_init()
{
    // Initialize I2C pins
    magtorq_init_pins();

    // Initialize I2C at 400kHz
    i2c_init(magtorq_i2c_inst, 400 * 1000);

    // Test communication by performing a dummy read on channel 0
    uint8_t command = magtorq_channel_commands[0];
    int8_t result = magtorq_write_i2c(&command, 1);

    if (result != 0)
    {
        magtorq_print_result(result, "initialization");
        return false;
    }

    // Small delay to allow ADC to settle
    sleep_ms(1);

    // Try to read a value to verify communication
    uint8_t dummy_value;
    result = magtorq_read_i2c(&dummy_value, 1);

    if (result != 0)
    {
        magtorq_print_result(result, "initialization read test");
        return false;
    }

    LOG_INFO("[magtorq-monitor] Magnetorquer monitor ADS7830 ADC initialized "
             "successfully");
    return true;
}

/**
 * @brief Read ADC value from specified magnetorquer channel
 *
 * @param channel   ADC channel (0-7)
 * @param value_out Pointer to store the 8-bit ADC reading
 * @return true if successful, false otherwise
 */
bool magtorq_get_reading(uint8_t channel, uint8_t *value_out)
{
    if (channel >= MAGTORQ_MAX_CHANNELS)
    {
        LOG_ERROR("[magtorq-monitor] Invalid ADC channel %d (must be 0-7)",
                  channel);
        return false;
    }

    if (value_out == NULL)
    {
        LOG_ERROR(
            "[magtorq-monitor] Null pointer provided for ADC value output");
        return false;
    }

    // Send channel selection command
    uint8_t command = magtorq_channel_commands[channel];
    int8_t result = magtorq_write_i2c(&command, 1);

    if (result != 0)
    {
        magtorq_print_result(result, "channel selection");
        return false;
    }

    // Allow ADC conversion time
    sleep_ms(1);

    // Read the converted value
    result = magtorq_read_i2c(value_out, 1);

    if (result != 0)
    {
        magtorq_print_result(result, "value read");
        return false;
    }

    return true;
}

/**
 * @brief Read ADC value and convert to current
 *
 * @param channel        ADC channel (0-7)
 * @param current_out    Pointer to store current (in amperes)
 * @param sense_resistor Current sense resistor value in ohms
 * @param amp_gain       Current sense amplifier gain (V/V)
 * @param vref           Reference voltage (typically 2.5V)
 * @return true if successful, false otherwise
 */
bool magtorq_get_current(uint8_t channel, float *current_out,
                         float sense_resistor, float amp_gain, float vref)
{
    if (current_out == NULL)
    {
        LOG_ERROR("[magtorq-monitor] Null pointer provided for current output");
        return false;
    }

    if (sense_resistor <= 0.0f || amp_gain <= 0.0f)
    {
        LOG_ERROR("[magtorq-monitor] Invalid sense resistor (%.3f) or "
                  "amplifier gain (%.3f)",
                  sense_resistor, amp_gain);
        return false;
    }

    uint8_t raw_value;
    if (!magtorq_get_reading(channel, &raw_value))
    {
        return false;
    }

    // Convert 8-bit ADC value to voltage
    float voltage = ((float)raw_value / (float)MAGTORQ_MAX_VALUE) * vref;

    // Convert voltage to current: I = V_adc / (R_sense * Gain)
    *current_out = voltage / (sense_resistor * amp_gain);

    return true;
}

/**
 * @brief Read all 8 magnetorquer ADC channels
 *
 * @param values_out Array to store 8 ADC readings
 * @return true if all channels read successfully, false otherwise
 */
bool magtorq_read_all_channels(uint8_t values_out[MAGTORQ_MAX_CHANNELS])
{
    if (values_out == NULL)
    {
        LOG_ERROR(
            "[magtorq-monitor] Null pointer provided for ADC values array");
        return false;
    }

    for (uint8_t channel = 0; channel < MAGTORQ_MAX_CHANNELS; channel++)
    {
        if (!magtorq_get_reading(channel, &values_out[channel]))
        {
            LOG_ERROR("[magtorq-monitor] Failed to read ADC channel %d",
                      channel);
            return false;
        }

        // Small delay between channel reads
        sleep_us(500);
    }

    return true;
}

/**
 * @brief Read all 8 magnetorquer channels and convert to currents
 *
 * @param currents_out   Array to store 8 current readings (in amperes)
 * @param sense_resistor Current sense resistor value in ohms
 * @param amp_gain       Current sense amplifier gain (V/V)
 * @param vref           Reference voltage (typically 2.5V)
 * @return true if all channels read successfully, false otherwise
 */
bool magtorq_read_all_currents(float currents_out[MAGTORQ_MAX_CHANNELS],
                               float sense_resistor, float amp_gain, float vref)
{
    if (currents_out == NULL)
    {
        LOG_ERROR("[magtorq-monitor] Null pointer provided for currents array");
        return false;
    }

    for (uint8_t channel = 0; channel < MAGTORQ_MAX_CHANNELS; channel++)
    {
        if (!magtorq_get_current(channel, &currents_out[channel],
                                 sense_resistor, amp_gain, vref))
        {
            LOG_ERROR("[magtorq-monitor] Failed to read current on channel %d",
                      channel);
            return false;
        }

        // Small delay between channel reads
        sleep_us(500);
    }

    return true;
}

/**
 * @brief Get magnetorquer current vector (X, Y, Z axes)
 *
 * @param current_vector 3D vector to store current readings [X, Y, Z] in
 * amperes
 * @param sense_resistor Current sense resistor value in ohms
 * @param amp_gain       Current sense amplifier gain (V/V)
 * @param vref           Reference voltage (typically 2.5V)
 * @return true if all axes read successfully, false otherwise
 */
bool magtorq_get_current_vector(float3 &current_vector, float sense_resistor,
                                float amp_gain, float vref)
{
    // Read X, Y, Z magnetorquer currents (channels 0, 1, 2)
    for (int axis = 0; axis < 3; axis++)
    {
        if (!magtorq_get_current(axis, &current_vector[axis], sense_resistor,
                                 amp_gain, vref))
        {
            LOG_ERROR("[magtorq-monitor] Failed to read current for axis %d",
                      axis);
            return false;
        }
    }

    return true;
}