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

static bool watchdog_initialized = false;
static bool pin_high = false;
static absolute_time_t pin_high_time;
static absolute_time_t last_feed_time;

void watchdog_init(void)
{
    gpio_init(SAMWISE_ADCS_WATCHDOG_FEED);
    gpio_set_dir(SAMWISE_ADCS_WATCHDOG_FEED, GPIO_OUT);
    gpio_put(SAMWISE_ADCS_WATCHDOG_FEED, 0);

    watchdog_initialized = true;
    pin_high = false;
    pin_high_time = nil_time;
    last_feed_time = get_absolute_time();
}

void watchdog_feed(void)
{
    // LOG_DEBUG("[watchdog] Feeding watchdog");
    if (!watchdog_initialized)
    {
        LOG_ERROR("[watchdog] Watchdog not initialized!");
        return;
    }

    absolute_time_t now = get_absolute_time();

    // Handle pin timing - pull low after 200ms if currently high
    if (pin_high && absolute_time_diff_us(pin_high_time, now) >= 200000)
    {
        LOG_DEBUG("[watchdog] wdt LOW");
        gpio_put(SAMWISE_ADCS_WATCHDOG_FEED, 0);
        pin_high = false;
        pin_high_time = nil_time;
    }

    // Feed watchdog if 2 seconds have passed and pin is not currently high
    if (!pin_high && absolute_time_diff_us(last_feed_time, now) >= 2000000)
    {
        LOG_DEBUG("[watchdog] wdt HIGH");
        gpio_put(SAMWISE_ADCS_WATCHDOG_FEED, 1);
        pin_high = true;
        pin_high_time = now;
        last_feed_time = now;
    }
}