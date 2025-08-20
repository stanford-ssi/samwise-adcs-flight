/**
 * @author Lundeen Cahilly
 * @date 2025-08-20
 *
 * This file reads data from the YZ (+-) photodiodes
 * using the ADC pins on a RP2350b chip. These support
 * 12-bit ADC resolution and a reference voltage of 3.3V.
 */

#include "photodiodes_yz.h"

#include "constants.h"
#include "macros.h"
#include "pins.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

constexpr uint8_t SAMWISE_ADCS_ADC_CHANNELS[8] = {
    SAMWISE_ADCS_PD_YP1, SAMWISE_ADCS_PD_YP2, SAMWISE_ADCS_PD_ZP1,
    SAMWISE_ADCS_PD_ZP2, SAMWISE_ADCS_PD_YM1, SAMWISE_ADCS_PD_YM2,
    SAMWISE_ADCS_PD_ZM1, SAMWISE_ADCS_PD_ZM2};

constexpr float VREF_YZ = 3.3f;        // Reference voltage for ADC conversion
constexpr uint8_t ADC_RESOLUTION = 12; // 12-bit ADC resolution
constexpr uint16_t ADC_MAX_VALUE = (1 << ADC_RESOLUTION); // 4095 for 12-bit ADC

/**
 * Initialize the YZ sun sensors by setting up the ADC hardware
 * @return true if initialization successful, false otherwise
 */
bool photodiodes_yz_init(void)
{
    // Make sure photodiodes are enabled
    gpio_init(SAMWISE_ADCS_EN_PD);
    gpio_set_dir(SAMWISE_ADCS_EN_PD, GPIO_OUT);
    gpio_put(SAMWISE_ADCS_EN_PD, 0); // Pull low to enable photodiode

    // Initialize ADC pins
    adc_init();

    for (uint8_t channel = 0; channel < 8; channel++)
    {
        adc_gpio_init(SAMWISE_ADCS_ADC_CHANNELS[channel]);
    }

    return true;
}

/**
 * Get a raw ADC reading from a specific sun sensor channel
 * @param channel ADC channel number (0-3 for Y+, Y-, Z+, Z-)
 * @param value Pointer to store the 12-bit ADC value (0-4095)
 * @return true if reading successful, false otherwise
 */
bool photodiodes_yz_get_reading(uint8_t channel, uint16_t *value)
{
    if (channel >= 8 || value == NULL)
    {
        LOG_ERROR("[photodiodes_yz] Invalid channel or null pointer for value");
        return false;
    }

    // Select ADC channel
    adc_select_input(channel);

    // Add small delay for ADC settling
    // sleep_us(10);

    // Read the ADC value - return full 12-bit resolution
    uint16_t result = adc_read();
    *value = result;

    if (*value > ADC_MAX_VALUE)
    {
        LOG_ERROR("[photodiodes_yz] ADC value out of range: %d", *value);
        return false;
    }

    return true;
}

/**
 * Get a voltage reading from a specific sun sensor channel
 * @param channel ADC channel number (0-3 for Y+, Y-, Z+, Z-)
 * @param voltage Pointer to store the calculated voltage
 * @return true if reading successful, false otherwise
 */
bool photodiodes_yz_get_voltage(uint8_t channel, float *voltage)
{
    if (channel >= 8 || voltage == NULL)
    {
        LOG_ERROR(
            "[photodiodes_yz] Invalid channel or null pointer for voltage");
        return false;
    }

    uint16_t value;
    if (!photodiodes_yz_get_reading(channel, &value))
    {
        LOG_ERROR("[photodiodes_yz] Failed to get reading for channel %d",
                  channel);
        return false;
    }

    // Convert 12-bit ADC value to voltage
    *voltage =
        ((float)value / (float)ADC_MAX_VALUE) * VREF_YZ; // Scale to 0-3.3V

    if (*voltage < 0.0f || *voltage > VREF_YZ)
    {
        LOG_ERROR("[photodiodes_yz] Voltage out of range: %f", *voltage);
        return false;
    }

    return true;
}

/**
 * Read all 8 sun sensor channels and return raw ADC values
 * @param values_out Array to store 8-bit ADC values for all channels
 * @return true if all readings successful, false otherwise
 */
bool photodiodes_yz_read_all_channels(uint16_t values_out[8])
{
    if (values_out == NULL)
    {
        LOG_ERROR(
            "[photodiodes_yz] Null pointer provided for ADC values array");
        return false;
    }

    for (uint8_t channel = 0; channel < 8; channel++)
    {
        if (!photodiodes_yz_get_reading(channel, &values_out[channel]))
        {
            LOG_ERROR("[photodiodes_yz] Failed to read ADC channel %d",
                      channel);
            return false;
        }

        // Add small delay between channels to prevent potential ADC issues
        // sleep_us(100);
    }

    return true;
}

/**
 * Read all 8 sun sensor channels and return voltage values
 * @param voltages_out Array to store voltage values for all channels
 * @return true if all readings successful, false otherwise
 */
bool photodiodes_yz_read_all_voltages(float voltages_out[8])
{
    if (voltages_out == NULL)
    {
        LOG_ERROR("[photodiodes_yz] Null pointer provided for voltage array");
        return false;
    }

    for (uint8_t channel = 0; channel < 8; channel++)
    {
        if (!photodiodes_yz_get_voltage(channel, &voltages_out[channel]))
        {
            LOG_ERROR("[photodiodes_yz] Failed to read voltage for channel %d",
                      channel);
            return false;
        }

        // Add small delay between voltage reads
        // sleep_us(10);
    }

    return true;
}