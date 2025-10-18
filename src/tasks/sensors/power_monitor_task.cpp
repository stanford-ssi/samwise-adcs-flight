/**
 * @author Lundeen Cahilly
 * @date 2025-10-18
 *
 * Power monitor task for reading ADCS board power consumption.
 * Runs at 10 Hz (can be merged with magnetometer task if needed).
 */

#include "power_monitor_task.h"
#include "constants.h"
#include "macros.h"

#include "drivers/power_monitor/power_monitor.h"

/**
 * @brief Initialize power monitor task.
 *
 * @param slate Pointer to the current satellite slate
 */
void power_monitor_task_init(slate_t *slate)
{
    LOG_INFO("[sensor] Initializing ADCS Power Monitor...");
    bool adm1176_result = adm_init();
    slate->power_monitor_alive = adm1176_result;

    if (!adm1176_result)
    {
        LOG_ERROR(
            "[sensor] Error initializing ADCS Power Monitor - deactivating!");
        adm_power_off();
    }

    LOG_INFO("[sensor] ADCS Power Monitor Initialization Complete! "
             "Power Monitor alive: %s",
             slate->power_monitor_alive ? "true" : "false");
}

/**
 * @brief Dispatch power monitor task. Reads power/voltage/current.
 *
 * @param slate Pointer to the current satellite slate
 */
void power_monitor_task_dispatch(slate_t *slate)
{
    if (!slate->power_monitor_alive)
    {
        LOG_DEBUG("[sensor] Skipping ADCS Power Monitor due to invalid "
                  "initialization!");
        return;
    }

    bool result = adm_get_power(slate);
    LOG_DEBUG("[sensor] P = %.3fW, V = %.3fV, "
              "I = %.3fA]",
              slate->adcs_power, slate->adcs_voltage, slate->adcs_current);
}

sched_task_t power_monitor_task = {.name = "power_monitor",
                                   .dispatch_period_ms = 100, // 10 Hz
                                   .task_init = &power_monitor_task_init,
                                   .task_dispatch =
                                       &power_monitor_task_dispatch,
                                   .next_dispatch = 0};
