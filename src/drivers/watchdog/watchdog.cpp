/**
 * @author Lundeen Cahilly
 * @date 2025-08-13
 *
 * This file feeds the watchdog timer to prevent system resets.
 */

#include "watchdog.h"
#include "hardware/gpio.h"
#include "macros.h"
#include "pins.h"

constexpr uint32_t watchdog_pin_high_us = 200000;       // 200 ms
constexpr uint32_t watchdog_feed_interval_us = 2000000; // 2 seconds

/**
 * @brief Initialize the watchdog timer GPIO pin and state in the slate.
 *
 * @param slate Pointer to the slate
 * @return true if initialization was successful
 * @return false if initialization failed
 */
bool watchdog_init(slate_t *slate)
{
    gpio_init(SAMWISE_ADCS_WATCHDOG_FEED);
    gpio_set_dir(SAMWISE_ADCS_WATCHDOG_FEED, GPIO_OUT);
    gpio_put(SAMWISE_ADCS_WATCHDOG_FEED, 0);

    slate->watchdog_pin_high = false;
    slate->watchdog_pin_high_time = nil_time;
    slate->watchdog_last_pet_time = get_absolute_time();

    return slate->watchdog_alive = true;
}

/**
 * @brief Pet the watchdog to avoid reset. Should be called frequently (at least
 * once every 2 seconds).
 *
 * @param slate Pointer to the slate
 */
void watchdog_pet(slate_t *slate)
{
    if (!slate->watchdog_alive)
    {
        LOG_ERROR("[watchdog] Watchdog not initialized!");
        return;
    }

    absolute_time_t now = get_absolute_time();

    // Handle pin timing - pull low after 200ms if currently high
    if (slate->watchdog_pin_high &&
        absolute_time_diff_us(slate->watchdog_pin_high_time, now) >=
            watchdog_pin_high_us)
    {
        LOG_INFO("[watchdog] wdt LOW");
        gpio_put(SAMWISE_ADCS_WATCHDOG_FEED, 0);
        slate->watchdog_pin_high = false;
        slate->watchdog_pin_high_time = nil_time;
    }

    // Feed watchdog if 2 seconds have passed and pin is not currently high
    if (!slate->watchdog_pin_high &&
        absolute_time_diff_us(slate->watchdog_last_pet_time, now) >=
            watchdog_feed_interval_us)
    {
        LOG_INFO("[watchdog] wdt HIGH");
        gpio_put(SAMWISE_ADCS_WATCHDOG_FEED, 1);
        slate->watchdog_pin_high = true;
        slate->watchdog_pin_high_time = now;
        slate->watchdog_last_pet_time = now;
    }
}