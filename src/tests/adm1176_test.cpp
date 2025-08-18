/**
 * @author  Lundeen Cahilly
 * @date    2025-08-02
 *
 * Read ADM1176 power monitor
 */

#include "../drivers/adm1176.h"
#include "macros.h"
#include "pico/stdlib.h"

/**
 * @brief Initialize power monitoring and log initial power state
 *
 */
void init_power_monitor()
{
    LOG_DEBUG("[adm1176_test] Attempting to initialize ADM1176...");

    if (adm_init())
    {
        // Power on the device after initialization
        if (!adm_power_on())
        {
            LOG_ERROR("[adm1176_test] Failed to power on ADM1176 after "
                      "initialization");
            return;
        }

        float voltage, current;

        // Get initial power readings
        if (adm_get_voltage(&voltage) && adm_get_current(&current))
        {
            float power = voltage * current;
            LOG_INFO(
                "[adm1176_test] Initial power state: %.3f V, %.3f A, %.3f W",
                voltage, current, power);
        }
        else
        {
            LOG_INFO("[adm1176_test] ADM1176 initialized but unable to read "
                     "initial power");
        }
    }
    else
    {
        LOG_ERROR("[adm1176_test] Failed to initialize ADM1176 power monitor");
        LOG_DEBUG("[adm1176_test] Check I2C1 bus and ADM1176 address 0x%02X",
                  0x4A);
    }
}

/**
 * @brief Read power monitor and log current power state
 *
 */
void read_power_monitor()
{
    float voltage, current;

    if (adm_get_voltage(&voltage) && adm_get_current(&current))
    {
        float power = voltage * current;
        LOG_INFO("[adm1176_test] Current power state: %.3f V, %.3f A, %.3f W",
                 voltage, current, power);
    }
    else
    {
        LOG_INFO("[adm1176_test] Failed to read power state");
    }
}