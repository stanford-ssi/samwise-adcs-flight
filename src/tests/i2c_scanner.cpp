/**
 * @author  Lundeen Cahilly and Yiheng
 * @date    2025-07-20
 *
 * i2c scanner task from flight software
 */

#include "constants.h"
#include "macros.h"
#include "pins.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include <stdint.h>
#include <stdlib.h>

#define I2C_TIMEOUT_MS 1000 // Timeout for I2C operations in milliseconds

// Read the I2C bus for devices
// function protype below
void i2c_scanner(void)
{
    LOG_INFO("[i2c_scanner] Starting I2C scanner...");

    bool found_device = false;
    for (uint8_t addr = 0x08; addr < 0x78; ++addr)
    { // Valid 7-bit I2C addresses
        uint8_t rxdata;
        // try_lock equivalent: i2c_read_blocking checks for ACK
        // A read of 1 byte is a common way to check for a device
        // For some devices, a write is better. LT8491 should respond to a read
        // attempt to a valid address. However, a more robust check might
        // involve trying to read a known register. For a simple scan, we just
        // see if we get an ACK. The SDK functions return PICO_ERROR_GENERIC if
        // no device responds.
        LOG_INFO("Scanning IMU address 0x%02X\n", addr);
        int ret = i2c_read_blocking_until(SAMWISE_ADCS_IMU_I2C, addr, &rxdata,
                                          1, false,
                                          make_timeout_time_ms(I2C_TIMEOUT_MS));
        if (ret >= 0)
        { // If ret is not an error code (i.e., ACK received)
            LOG_INFO("IMU Device found at 0x%02X\n", addr);
            found_device = true;
        }
        else
        {
            LOG_INFO("No device at 0x%02X\n", addr);
        }
    }
    if (!found_device)
    {
        LOG_ERROR("No I2C devices found.\n");
    }
}