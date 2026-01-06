/**
**
 * @author The SSI Sats ADCS team
 *
 * This is the main file, where it all begins!
 */

#include "pico/stdlib.h"

#include "init.h"
#include "macros.h"
#include "motor_slate.h" // TODO: Make separate motor slate

// Make sure top gpio bank enabled
static_assert(PICO_RP2350A == 0,
              "PICO_RP2350A must be defined to 0 for PICUBED builds.");

/**
 * One slate to rule them all!
 */
motor_slate_t motor_slate;

int main()
{
    stdio_init_all();

    sleep_ms(5000);

    /*
     * Run global initialization
     */
    LOG_INFO("[main] Running global init...");
    // TODO: Motor board init
    //init(&slate);

    /*
     * Initialize the state machine
     */
    sleep_ms(1000);

    LOG_INFO("[main] Slate takes up %d bytes!", sizeof(motor_slate));
    //sched_init(&slate);

    /*
     * Run the state machine for all of time
     */
    LOG_INFO("[main] Initialization sequence complete - beginning main loop!");
    while (1)
    {
        sleep_ms(1000);
        LOG_INFO("[loop] running");
        //sched_dispatch(&slate);
    }

    // We should not be here -> very bad
    ERROR("Reached the end of the code! This should not happen!");
}
