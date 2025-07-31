/**
 * @author Sage Wu
 * @date 2025-06-03
 *
 * This file defines utilities for controlling the ADS7830 ADC.
 *
 * ADS7830 is an 8-bit, 8-channel ADC with I2C interface for sun sensor
 * readings.
 */

#include "ads7830.h"

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

/*! Channel selection commands for single-ended mode */
static const uint8_t ads7830_channel_commands[ADS7830_MAX_CHANNELS] = {
    0x8C, // Channel 0: (0x08 << 4) | 0x0C
    0xCC, // Channel 1: (0x0C << 4) | 0x0C
    0x9C, // Channel 2: (0x09 << 4) | 0x0C
    0xDC, // Channel 3: (0x0D << 4) | 0x0C
    0xAC, // Channel 4: (0x0A << 4) | 0x0C
    0xEC, // Channel 5: (0x0E << 4) | 0x0C
    0xBC, // Channel 6: (0x0B << 4) | 0x0C
    0xFC  // Channel 7: (0x0F << 4) | 0x0C
};

/*! I2C instance for ADC communication */
static i2c_inst_t *ads7830_i2c_inst = i2c1;

/******************************************************************************/
/*!                Static function definitions                                */

/**
 * Thin wrapper around writing to the I2C bus
 */
static int8_t ads7830_write_i2c(uint8_t *data, uint16_t count)
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
static int8_t ads7830_read_i2c(uint8_t *data, uint16_t count)
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
static void ads7830_print_result(int8_t rslt, const char *operation)
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
static void ads7830_init_pins()
{
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
bool ads7830_init()
{
    // Initialize I2C pins
    ads7830_init_pins();

    // Initialize I2C at 400kHz
    i2c_init(ads7830_i2c_inst, 400 * 1000);

    // Test communication by performing a dummy read on channel 0
    uint8_t command = ads7830_channel_commands[0];
    int8_t result = ads7830_write_i2c(&command, 1);

    if (result != 0)
    {
        ads7830_print_result(result, "initialization");
        return false;
    }

    // Small delay to allow ADC to settle
    sleep_ms(1);

    // Try to read a value to verify communication
    uint8_t dummy_value;
    result = ads7830_read_i2c(&dummy_value, 1);

    if (result != 0)
    {
        ads7830_print_result(result, "initialization read test");
        return false;
    }

    LOG_INFO("[ads7830] ADS7830 ADC initialized successfully");
    return true;
}

/**
 * @brief Read ADC value from specified channel
 *
 * @param channel   ADC channel (0-7)
 * @param value_out Pointer to store the 8-bit ADC reading
 * @return true if successful, false otherwise
 */
bool ads7830_read_channel(uint8_t channel, uint8_t *value_out)
{
    if (channel >= ADS7830_MAX_CHANNELS)
    {
        LOG_ERROR("[ads7830] Invalid ADC channel %d (must be 0-7)", channel);
        return false;
    }

    if (value_out == NULL)
    {
        LOG_ERROR("[ads7830] Null pointer provided for ADC value output");
        return false;
    }

    // Send channel selection command
    uint8_t command = ads7830_channel_commands[channel];
    int8_t result = ads7830_write_i2c(&command, 1);

    if (result != 0)
    {
        ads7830_print_result(result, "channel selection");
        return false;
    }

    // Allow ADC conversion time
    sleep_ms(1);

    // Read the converted value
    result = ads7830_read_i2c(value_out, 1);

    if (result != 0)
    {
        ads7830_print_result(result, "value read");
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
bool ads7830_read_voltage(uint8_t channel, float *voltage_out, float vref)
{
    if (voltage_out == NULL)
    {
        LOG_ERROR("[ads7830] Null pointer provided for voltage output");
        return false;
    }

    uint8_t raw_value;
    if (!ads7830_read_channel(channel, &raw_value))
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
bool ads7830_read_all_channels(uint8_t values_out[ADS7830_MAX_CHANNELS])
{
    if (values_out == NULL)
    {
        LOG_ERROR("[ads7830] Null pointer provided for ADC values array");
        return false;
    }

    for (uint8_t channel = 0; channel < ADS7830_MAX_CHANNELS; channel++)
    {
        if (!ads7830_read_channel(channel, &values_out[channel]))
        {
            LOG_ERROR("[ads7830] Failed to read ADC channel %d", channel);
            return false;
        }

        // Small delay between channel reads
        sleep_us(500);
    }

    return true;
}