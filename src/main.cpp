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

/**
 * One slate to rule them all!
 */
slate_t slate;

int main()
{
    stdio_init_all();

    /*
     * Initialize the state machine
     */
    sleep_ms(5000);

    LOG_INFO("[main] Slate takes up %d bytes!", sizeof(slate));
    sched_init(&slate);

    /*
     * Run global initialization
     */
    LOG_INFO("[main] Running global init...");
    init(&slate);

    /*
     * Run the state machine for all of time
     */
    LOG_INFO("[main] Initialization sequence complete - beginning main loop!");
    while (1)
    {
        sched_dispatch(&slate);
    }

    // We should not be here -> very bad
    ERROR("Reached the end of the code! This should not happen!");
}