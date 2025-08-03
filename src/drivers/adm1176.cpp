/**
 * @author Lundeen Cahilly
 * @date 2025-08-02
 *
 * Simple ADM1176 power monitor driver
 */

#include "adm1176.h"
#include "hardware/i2c.h"
#include "macros.h"
#include "pico/stdlib.h"
#include "pins.h"
#include <stdint.h>

// ADM1176 I2C configuration
#define ADM1176_I2C_ADDR 0x4A
#define I2C_TIMEOUT_MS 100 // Add this if not defined elsewhere

// Data masks
#define DATA_V_MASK 0xF0
#define DATA_I_MASK 0x0F

// Conversion constants
#define VOLTAGE_SCALE (26.35f / 4096.0f) // For VRANGE = 0
#define CURRENT_SCALE (0.10584f / 4096.0f)
#define DEFAULT_SENSE_RESISTOR 0.1f

// Static buffers (matching working driver)
static uint8_t _cmd_buf[1];
static uint8_t _ext_cmd_buf[2];
static uint8_t _read_buf[3];

// Static state
static bool is_initialized = false;
static float sense_resistor = DEFAULT_SENSE_RESISTOR;

/**
 * Configure ADM1176 modes (matches working driver exactly)
 */
static bool adm_config(int *mode, int mode_len)
{
    _cmd_buf[0] = 0x0;

    for (int i = 0; i < mode_len; i++)
    {
        switch (mode[i])
        {
            case 1:
                _cmd_buf[0] |= (1 << 0); // V_CONT
                break;
            case 2:
                _cmd_buf[0] |= (1 << 1); // V_ONCE
                break;
            case 3:
                _cmd_buf[0] |= (1 << 2); // I_CONT
                break;
            case 4:
                _cmd_buf[0] |= (1 << 3); // I_ONCE
                break;
            case 5:
                _cmd_buf[0] |= (1 << 4); // V_RANGE
                break;
        }
    }

    if (i2c_write_blocking_until(SAMWISE_ADCS_PWR_I2C, ADM1176_I2C_ADDR,
                                 _cmd_buf, 1, false,
                                 make_timeout_time_ms(I2C_TIMEOUT_MS)) != 1)
    {
        return false;
    }

    return true;
}

/**
 * Initialize ADM1176 power monitor (simple initialization, no device config)
 * @param resistor_ohms Sense resistor value in ohms (use 0.0f for default)
 * @return true if initialization successful, false otherwise
 */
bool adm_init(float resistor_ohms)
{
    if (resistor_ohms > 0.0f)
    {
        sense_resistor = resistor_ohms;
    }

    is_initialized = true;
    LOG_INFO("ADM1176 initialized on I2C1 with sense resistor: %.3f ohms",
             sense_resistor);

    return true;
}

/**
 * Turn on ADM1176 power monitoring (matches working driver exactly)
 * @return true if successful, false otherwise
 */
bool adm_power_on(void)
{
    if (!is_initialized)
    {
        LOG_ERROR("ADM1176 not initialized");
        return false;
    }

    // Extended command to turn on (matches working driver)
    _ext_cmd_buf[0] = 0x83;
    _ext_cmd_buf[1] = 0;

    LOG_DEBUG("ADM1176: Sending power-on command 0x83, 0x00 to address 0x%02X",
              ADM1176_I2C_ADDR);

    int result = i2c_write_blocking_until(
        SAMWISE_ADCS_PWR_I2C, ADM1176_I2C_ADDR, _ext_cmd_buf, 2, false,
        make_timeout_time_ms(I2C_TIMEOUT_MS));

    LOG_DEBUG("ADM1176: I2C write result: %d (expected: 2)", result);

    if (result != 2)
    {
        LOG_ERROR("Failed to turn on ADM1176, I2C write returned: %d", result);
        return false;
    }

    // Configure for continuous voltage and current (matches working driver)
    int modes[2] = {1, 3}; // V_CONT and I_CONT
    if (!adm_config(modes, 2))
    {
        LOG_ERROR("Failed to configure ADM1176 after power on");
        return false;
    }

    LOG_DEBUG("ADM1176 powered on, config: 0x%02X", _cmd_buf[0]);
    return true;
}

/**
 * Turn off ADM1176 power monitoring (matches working driver exactly)
 * @return true if successful, false otherwise
 */
bool adm_power_off(void)
{
    if (!is_initialized)
    {
        LOG_ERROR("ADM1176 not initialized");
        return false;
    }

    // Extended command to turn off (matches working driver)
    _ext_cmd_buf[0] = 0x83;
    _ext_cmd_buf[1] = 1;
    if (i2c_write_blocking_until(SAMWISE_ADCS_PWR_I2C, ADM1176_I2C_ADDR,
                                 _ext_cmd_buf, 2, false,
                                 make_timeout_time_ms(I2C_TIMEOUT_MS)) < 0)
    {
        LOG_ERROR("Failed to turn off ADM1176");
        return false;
    }

    LOG_DEBUG("ADM1176 powered off");
    return true;
}

/**
 * Get voltage measurement in volts (matches working driver exactly)
 * @param voltage_out Pointer to store voltage value
 * @return true if successful, false otherwise
 */
bool adm_get_voltage(float *voltage_out)
{
    if (!is_initialized || !voltage_out)
    {
        LOG_ERROR("ADM1176 not initialized or invalid parameter");
        return false;
    }

    // Turn on and wait (matches working driver)
    if (!adm_power_on())
    {
        return false;
    }
    sleep_ms(1);

    // Read 3 bytes (matches working driver)
    if (i2c_read_blocking_until(SAMWISE_ADCS_PWR_I2C, ADM1176_I2C_ADDR,
                                _read_buf, 3, false,
                                make_timeout_time_ms(I2C_TIMEOUT_MS)) != 3)
    {
        LOG_ERROR("Failed to read voltage data from ADM1176");
        return false;
    }

    // Extract voltage data (matches working driver exactly)
    float raw_volts = ((_read_buf[0] << 8) | (_read_buf[2] & DATA_V_MASK)) >> 4;
    *voltage_out = VOLTAGE_SCALE * raw_volts;

    LOG_DEBUG("ADM1176 voltage: %.3f V (raw: %.0f)", *voltage_out, raw_volts);
    return true;
}

/**
 * Get current measurement in amps (matches working driver exactly)
 * @param current_out Pointer to store current value
 * @return true if successful, false otherwise
 */
bool adm_get_current(float *current_out)
{
    if (!is_initialized || !current_out)
    {
        LOG_ERROR("ADM1176 not initialized or invalid parameter");
        return false;
    }

    // Turn on and wait (matches working driver)
    if (!adm_power_on())
    {
        return false;
    }
    sleep_ms(1);

    // Read 3 bytes (matches working driver)
    if (i2c_read_blocking_until(SAMWISE_ADCS_PWR_I2C, ADM1176_I2C_ADDR,
                                _read_buf, 3, false,
                                make_timeout_time_ms(I2C_TIMEOUT_MS)) != 3)
    {
        LOG_ERROR("Failed to read current data from ADM1176");
        return false;
    }

    // Extract current data (matches working driver exactly)
    float raw_amps = ((_read_buf[0] << 8) | (_read_buf[2] & DATA_V_MASK)) >> 4;
    *current_out = (CURRENT_SCALE * raw_amps) / sense_resistor;

    LOG_DEBUG("ADM1176 current: %.3f A (raw: %.0f)", *current_out, raw_amps);
    return true;
}

/**
 * Read ADM1176 status register (matches working driver exactly)
 * @param status_out Pointer to store status value
 * @return true if successful, false otherwise
 */
bool adm_get_status(uint8_t *status_out)
{
    if (!is_initialized || !status_out)
    {
        LOG_ERROR("ADM1176 not initialized or invalid parameter");
        return false;
    }

    // Write status read command (matches working driver)
    uint8_t cmd = (1 << 6); // C6 = 1 → STATUS_RD
    if (i2c_write_blocking_until(SAMWISE_ADCS_PWR_I2C, ADM1176_I2C_ADDR, &cmd,
                                 1, false,
                                 make_timeout_time_ms(I2C_TIMEOUT_MS)) != 1)
    {
        LOG_ERROR("Failed to request status from ADM1176");
        return false;
    }

    // Read status byte (matches working driver)
    if (i2c_read_blocking_until(SAMWISE_ADCS_PWR_I2C, ADM1176_I2C_ADDR,
                                status_out, 1, false,
                                make_timeout_time_ms(I2C_TIMEOUT_MS)) != 1)
    {
        LOG_ERROR("Failed to read status from ADM1176");
        return false;
    }

    LOG_DEBUG("ADM1176 status: 0x%02X", *status_out);
    return true;
}

/**
 * Check if ADM1176 driver is initialized
 * @return true if initialized, false otherwise
 */
bool adm_is_initialized(void)
{
    return is_initialized;
}