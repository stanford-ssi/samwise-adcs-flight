/**
 * @author Lundeen Cahilly
 * @date 2025-08-20
 *
 * This file reads data from the sun pyramid sensors
 * using the ADC pins on a RP2350b chip. These support
 * 12-bit ADC resolution and a reference voltage of 3.3V.
 *
 * Board v1.8: Sun pyramids connected to RP2350B ADC (GPIO 40-47)
 */

#include "rp2350b_adc.h"

#include "constants.h"
#include "pins.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

constexpr uint8_t SAMWISE_ADCS_ADC_CHANNELS[8] = {
    SAMWISE_ADCS_PYRAMID1_1, SAMWISE_ADCS_PYRAMID1_2, SAMWISE_ADCS_PYRAMID1_3,
    SAMWISE_ADCS_PYRAMID1_4, SAMWISE_ADCS_PYRAMID2_1, SAMWISE_ADCS_PYRAMID2_2,
    SAMWISE_ADCS_PYRAMID2_3, SAMWISE_ADCS_PYRAMID2_4};

/**
 * Initialize the sun pyramid sensors by setting up the ADC hardware
 * @return true if initialization successful, false otherwise
 */
bool rp2350b_adc_init(void)
{
    // Make sure sun sensors are enabled
    gpio_init(SAMWISE_ADCS_EN_PD);
    gpio_set_dir(SAMWISE_ADCS_EN_PD, GPIO_OUT);
    gpio_put(SAMWISE_ADCS_EN_PD, 0); // Pull low to enable sun sensors

    // Initialize ADC pins
    adc_init();

    for (uint8_t channel = 0; channel < 8; channel++)
    {
        adc_gpio_init(SAMWISE_ADCS_ADC_CHANNELS[channel]);
    }

    return true;
}

/**
 * Get a raw ADC reading from a specific sun pyramid sensor channel
 * @param channel ADC channel number (0-7 for pyramid sensors)
 * @param value Pointer to store the 12-bit ADC value (0-4095)
 * @return true if reading successful, false otherwise
 */
bool rp2350b_adc_get_reading(uint8_t channel, uint16_t *value)
{
    if (channel >= 8 || value == NULL)
    {
        LOG_ERROR("[rp2350b_adc] Invalid channel or null pointer for value");
        return false;
    }

    // Select ADC channel
    adc_select_input(channel);

    // Add small delay for ADC settling
    // sleep_us(10);

    // Read the ADC value - return full 12-bit resolution
    uint16_t result = adc_read();
    *value = result;

    if (*value > MAX_VALUE_RP2350B_ADC)
    {
        LOG_ERROR("[rp2350b_adc] ADC value out of range: %d", *value);
        return false;
    }

    return true;
}

/**
 * Get a voltage reading from a specific sun pyramid sensor channel
 * @param channel ADC channel number (0-7 for pyramid sensors)
 * @param voltage Pointer to store the calculated voltage
 * @return true if reading successful, false otherwise
 */
bool rp2350b_adc_get_voltage(uint8_t channel, float *voltage)
{
    if (channel >= 8 || voltage == NULL)
    {
        LOG_ERROR("[rp2350b_adc] Invalid channel or null pointer for voltage");
        return false;
    }

    uint16_t value;
    if (!rp2350b_adc_get_reading(channel, &value))
    {
        LOG_ERROR("[rp2350b_adc] Failed to get reading for channel %d",
                  channel);
        return false;
    }

    // Convert 12-bit ADC value to voltage
    *voltage = ((float)value / (float)MAX_VALUE_RP2350B_ADC) *
               VREF_RP2350B_ADC; // Scale to 0-3.3V

    if (*voltage < 0.0f || *voltage > VREF_RP2350B_ADC)
    {
        LOG_ERROR("[rp2350b_adc] Voltage out of range: %f", *voltage);
        return false;
    }

    return true;
}

/**
 * Read all 8 sun pyramid sensor channels and return raw ADC values
 * @param values_out Array to store 12-bit ADC values for all channels
 * @return true if all readings successful, false otherwise
 */
bool rp2350b_adc_read_all_channels(uint16_t values_out[8])
{
    if (values_out == NULL)
    {
        LOG_ERROR("[rp2350b_adc] Null pointer provided for ADC values array");
        return false;
    }

    for (uint8_t channel = 0; channel < 8; channel++)
    {
        if (!rp2350b_adc_get_reading(channel, &values_out[channel]))
        {
            LOG_ERROR("[rp2350b_adc] Failed to read ADC channel %d", channel);
            return false;
        }

        // Add small delay between channels to prevent potential ADC issues
        // sleep_us(100);
    }

    return true;
}

/**
 * Read all 8 sun pyramid sensor channels and return voltage values
 * @param voltages_out Array to store voltage values for all channels
 * @return true if all readings successful, false otherwise
 */
bool rp2350b_adc_read_all_voltages(float voltages_out[8])
{
    if (voltages_out == NULL)
    {
        LOG_ERROR("[rp2350b_adc] Null pointer provided for voltage array");
        return false;
    }

    for (uint8_t channel = 0; channel < 8; channel++)
    {
        if (!rp2350b_adc_get_voltage(channel, &voltages_out[channel]))
        {
            LOG_ERROR("[rp2350b_adc] Failed to read voltage for channel %d",
                      channel);
            return false;
        }

        // Add small delay between voltage reads
        // sleep_us(10);
    }

    return true;
}