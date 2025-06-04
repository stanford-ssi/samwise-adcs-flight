#include "ads7830.h"
#include <stdio.h>

#define I2C_TIMEOUT_MS 10

// Channel selection for single-ended mode
static const uint8_t CHANNEL_SELECTION[8] = {
    0x08, // SINGLE_CH0
    0x0C, // SINGLE_CH1
    0x09, // SINGLE_CH2
    0x0D, // SINGLE_CH3
    0x0A, // SINGLE_CH4
    0x0E, // SINGLE_CH5
    0x0B, // SINGLE_CH6
    0x0F  // SINGLE_CH7
};

// Initialize the ADS7830 ADC
uint8_t init_ADC(i2c_inst_t *i2c) {
    // No specific initialization is required for ADS7830
    // Test I2C communication by writing a dummy command. Select channel 0
    uint8_t dummy_command = CHANNEL_SELECTION[0] << 4  |
      (0b11 << 2); // Set PD1 and PD0 to 0b1, these turn on conversion and Vref

    absolute_time_t timeout = make_timeout_time_ms(I2C_TIMEOUT_MS);
    int result = i2c_write_blocking_until(i2c, ADS7830_I2C_ADDR, &dummy_command, 1, false, timeout);

    if (result == PICO_ERROR_GENERIC || result == PICO_ERROR_TIMEOUT) {
        return PICO_ERROR_GENERIC; // Error
    }

    return PICO_OK; // Success
}
// Read ADC value from a specific channel
uint8_t read_ADC(i2c_inst_t *i2c, uint8_t channel, uint8_t *adc_voltage) {
    if (channel > 7) {
        printf("Invalid channel: must be 0-7\n");
        return PICO_ERROR_GENERIC; // Invalid channel
    }

    // Prepare the command byte
    uint8_t command_byte = (CHANNEL_SELECTION[channel] << 4) |
       (0b11 << 2); // Set PD1 and PD0 to 0b1

    // Perform I2C write and read with timeout
    absolute_time_t timeout = make_timeout_time_ms(I2C_TIMEOUT_MS);
    int write_result = i2c_write_blocking_until(i2c, ADS7830_I2C_ADDR, &command_byte, 1, false, timeout);
    if (write_result == PICO_ERROR_GENERIC || write_result == PICO_ERROR_TIMEOUT) {
        return PICO_ERROR_GENERIC; // Error during write
    }
    sleep_ms(1); // Allow time for the ADC to settle
    int read_result = i2c_read_blocking_until(i2c, ADS7830_I2C_ADDR, adc_voltage, 1, false, timeout);
    if (read_result == PICO_ERROR_GENERIC || read_result == PICO_ERROR_TIMEOUT) {
        return PICO_ERROR_GENERIC; // Error during read
    }

    return PICO_OK; // Success
}