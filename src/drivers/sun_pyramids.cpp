/**
 * @author Lundeen Cahilly and Sage Wu
 * @date 2025-06-03
 *
 * This file defines utilities for controlling the ADS7830 ADC.
 *
 * ADS7830 is an 8-bit, 8-channel ADC with I2C interface for sun pyramids
 * readings.
 */

#include "sun_pyramids.h"

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

/*! I2C Device address */
#define ADS7830_I2C_ADDR (0x48)

/*! I2C timeout in milliseconds */
#define ADS7830_I2C_TIMEOUT_MS (10)

/*! Maximum ADC channels */
#define ADS7830_MAX_CHANNELS (8)

/*! Expected 8-bit ADC resolution */
#define ADS7830_RESOLUTION_BITS (8)
#define ADS7830_MAX_VALUE (255)

/******************************************************************************/
/*!                Static variable definition                                 */

// Using external reference
// static const uint8_t ads7830_channel_commands[8] = {
//     0x84, 0xC4, 0x94, 0xD4, 0xA4, 0xE4, 0xB4, 0xF4 // PD1=0, PD0=1
// };
// #define VREF \
//     (2.5f) // Reference voltage for external reference (confirm w/ hardware)

// Using internal reference (2.5V)
static const uint8_t ads7830_channel_commands[8] = {
    0x8C, 0xCC, 0x9C, 0xDC, 0xAC, 0xEC, 0xBC, 0xFC // PD1=1, PD0=1
};
#define VREF (2.5f) // Reference voltage for internal reference

/*! I2C instance for ADC communication */
static i2c_inst_t *ads7830_i2c_inst = i2c1;

/******************************************************************************/
/*!                Static function definitions                                */

/**
 * Thin wrapper around writing to the I2C bus
 */
static int8_t sun_pyramids_write_i2c(uint8_t *data, uint16_t count)
{
    if (count < 1)
    {
        return -1;
    }

    absolute_time_t timeout = make_timeout_time_ms(ADS7830_I2C_TIMEOUT_MS);
    int result = i2c_write_blocking_until(ads7830_i2c_inst, ADS7830_I2C_ADDR,
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
static int8_t sun_pyramids_read_i2c(uint8_t *data, uint16_t count)
{
    if (count < 1)
    {
        return -1;
    }

    absolute_time_t timeout = make_timeout_time_ms(ADS7830_I2C_TIMEOUT_MS);
    int result = i2c_read_blocking_until(ads7830_i2c_inst, ADS7830_I2C_ADDR,
                                         data, count, false, timeout);

    if (result == PICO_ERROR_GENERIC || result == PICO_ERROR_TIMEOUT)
    {
        return -1;
    }

    return 0;
}

/*!
 *  @brief Prints the execution status of ADC operations.
 */
static void sun_pyramids_print_result(int8_t rslt, const char *operation)
{
    if (rslt != 0)
    {
        LOG_ERROR("[ads7830] ADS7830 %s failed with result: %d", operation,
                  rslt);
    }
}

/******************************************************************************/
/*!                Public function definitions                                */

/**
 * Initialize I2C pins for ADS7830 communication
 */
static void sun_pyramids_init_pins(void)
{
    // Enable photodiode
    gpio_init(SAMWISE_ADCS_EN_PD);
    gpio_set_dir(SAMWISE_ADCS_EN_PD, GPIO_OUT);
    gpio_put(SAMWISE_ADCS_EN_PD, 0); // Pull low to enable photodiode

    // Initialize I2C1 pins
    gpio_init(SAMWISE_ADCS_I2C1_SDA);
    gpio_init(SAMWISE_ADCS_I2C1_SCL);
    gpio_set_function(SAMWISE_ADCS_I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(SAMWISE_ADCS_I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SAMWISE_ADCS_I2C1_SDA);
    gpio_pull_up(SAMWISE_ADCS_I2C1_SCL);
}

/**
 * Initialize the ADS7830 ADC
 */
bool sun_pyramids_init(void)
{
    // Initialize I2C pins
    sun_pyramids_init_pins();

    // Initialize I2C at 400kHz
    i2c_init(ads7830_i2c_inst, 400 * 1000);

    // Test communication by performing a dummy read on channel 0
    uint8_t command = ads7830_channel_commands[0];
    int8_t result = sun_pyramids_write_i2c(&command, 1);

    if (result != 0)
    {
        sun_pyramids_print_result(result, "initialization");
        return false;
    }

    // Small delay to allow ADC to settle
    // sleep_us(10);

    // Try to read a value to verify communication
    uint8_t dummy_value;
    result = sun_pyramids_read_i2c(&dummy_value, 1);

    if (result != 0)
    {
        sun_pyramids_print_result(result, "initialization read test");
        return false;
    }

    LOG_INFO("[sun_pyramids] ADS7830 ADC initialized successfully");
    return true;
}

/**
 * @brief Read ADC value from specified channel
 *
 * @param channel   ADC channel (0-7)
 * @param value_out Pointer to store the 8-bit ADC reading
 * @return true if successful, false otherwise
 */
bool sun_pyramids_read_channel(uint8_t channel, uint8_t *value_out)
{
    if (channel >= ADS7830_MAX_CHANNELS)
    {
        LOG_ERROR("[sun_pyramids] Invalid ADC channel %d (must be 0-7)",
                  channel);
        return false;
    }

    if (value_out == NULL)
    {
        LOG_ERROR("[sun_pyramids] Null pointer provided for ADC value output");
        return false;
    }

    // Send channel selection command
    uint8_t command = ads7830_channel_commands[channel];
    int8_t result = sun_pyramids_write_i2c(&command, 1);

    if (result != 0)
    {
        sun_pyramids_print_result(result, "channel selection");
        return false;
    }

    // Allow ADC conversion time
    // sleep_us(10);

    // Read the converted value
    result = sun_pyramids_read_i2c(value_out, 1);

    if (result != 0)
    {
        sun_pyramids_print_result(result, "value read");
        return false;
    }

    return true;
}

/**
 * @brief Read ADC value and convert to voltage
 *
 * @param channel     ADC channel (0-7)
 * @param voltage_out Pointer to store voltage (in volts)
 * @param vref        Reference voltage (typically 3.3V)
 * @return true if successful, false otherwise
 */
bool sun_pyramids_read_voltage(uint8_t channel, float *voltage_out, float vref)
{
    if (voltage_out == NULL)
    {
        LOG_ERROR("[sun_pyramids] Null pointer provided for voltage output");
        return false;
    }

    uint8_t raw_value;
    if (!sun_pyramids_read_channel(channel, &raw_value))
    {
        return false;
    }

    // Convert 8-bit ADC value to voltage
    *voltage_out = ((float)raw_value / (float)ADS7830_MAX_VALUE) * vref;

    return true;
}

/**
 * @brief Read all 8 ADC channels
 *
 * @param values_out Array to store 8 ADC readings
 * @return true if all channels read successfully, false otherwise
 */
bool sun_pyramids_read_all_channels(uint8_t values_out[ADS7830_MAX_CHANNELS])
{
    if (values_out == NULL)
    {
        LOG_ERROR("[sun_pyramids] Null pointer provided for ADC values array");
        return false;
    }

    for (uint8_t channel = 0; channel < ADS7830_MAX_CHANNELS; channel++)
    {
        if (!sun_pyramids_read_channel(channel, &values_out[channel]))
        {
            LOG_ERROR("[sun_pyramids] Failed to read ADC channel %d", channel);
            return false;
        }

        // Small delay between channel reads
        // sleep_us(10);
    }

    return true;
}

/**
 * @brief Read all 8 ADC channels and convert to voltages
 *
 * @param voltages_out Array to store 8 voltage readings (in volts)
 * @return true if all channels read successfully, false otherwise
 */
bool sun_pyramids_read_all_voltages(float voltages_out[ADS7830_MAX_CHANNELS])
{
    if (voltages_out == NULL)
    {
        LOG_ERROR("[sun_pyramids] Null pointer provided for voltages array");
        return false;
    }

    for (uint8_t channel = 0; channel < ADS7830_MAX_CHANNELS; channel++)
    {
        if (!sun_pyramids_read_voltage(channel, &voltages_out[channel], VREF))
        {
            LOG_ERROR("[sun_pyramids] Failed to read voltage on channel %d",
                      channel);
            return false;
        }

        // Small delay between channel reads
        // sleep_us(10);
    }

    return true;
}