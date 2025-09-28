/**
 * @author  Lundeen Cahilly and Yiheng
 * @date    2025-07-20
 *
 * i2c scanner task from flight software
 */

#include "i2c_scanner.h"
#include "constants.h"
#include "macros.h"
#include "pins.h"

#include "pico/stdlib.h"
#include "pico/time.h"

#include <stdint.h>
#include <stdlib.h>

/**
 * Scan I2C bus for responding devices
 * @param i2c_inst The I2C instance to scan
 * @param bus_name Name for logging (e.g., "I2C0", "I2C1")
 */
void scan_i2c_bus(i2c_inst_t *i2c_inst, const char *bus_name)
{
    LOG_INFO("Scanning %s bus for devices...", bus_name);

    bool found_device = false;
    uint8_t test_data = 0x00;

    for (uint8_t addr = 0x08; addr <= 0x77; addr++)
    {
        // Try to write 1 byte and read back - more reliable test
        int result = i2c_write_blocking(i2c_inst, addr, &test_data, 1, true);

        if (result == 1)
        {
            // Try to read back to confirm it's a real device
            uint8_t read_back;
            int read_result =
                i2c_read_blocking(i2c_inst, addr, &read_back, 1, false);

            if (read_result == 1)
            {
                LOG_INFO("Confirmed device at address 0x%02X", addr);
                found_device = true;
            }
        }
    }

    if (!found_device)
    {
        LOG_INFO("No devices found on %s bus - checking bus health...");

        // Test if SDA/SCL are stuck
        LOG_INFO(
            "I2C%d SDA pin %d, SCL pin %d", i2c_inst == i2c0 ? 0 : 1,
            i2c_inst == i2c0 ? SAMWISE_ADCS_I2C0_SDA : SAMWISE_ADCS_I2C1_SDA,
            i2c_inst == i2c0 ? SAMWISE_ADCS_I2C0_SCL : SAMWISE_ADCS_I2C1_SCL);
    }
}