/**
 * @author The SSI Sats ADCS team
 *
 * This is the main file, where it all begins!
 */

#include "pico/stdlib.h"

#include "init.h"
#include "macros.h"
#include "scheduler/scheduler.h"
#include "slate.h"

#include "pins.h"

/**
 * One slate to rule them all!
 */
slate_t slate;

int main()
{
    stdio_init_all();

    while (1)
    {
        LOG_INFO("[main] WAX ON...");
        gpio_put(SAMWISE_ADCS_NEOPIXEL, 1); // LED on
        sleep_ms(500);
        LOG_INFO("[main] WAX OFF...");
        gpio_put(SAMWISE_ADCS_NEOPIXEL, 0); // LED off
        sleep_ms(500);
    }

    // We should not be here -> very bad
    ERROR("Reached the end of the code! This should not happen!");
}