/**
 * @author The SSI Sats ADCS team
 *
 * This is the main file, where it all begins!
 */

#include "pico/stdlib.h"

#include "gnc/attitude_filter.h"
#include "gnc/bdot.h"
#include "gnc/matrix_utils.h"
#include "gnc/reaction_wheel_allocation.h"
#include "gnc/sun_pyramid_reading.h"
#include "gnc/sun_vector.h"
#include "init.h"
#include "macros.h"
#include "scheduler/scheduler.h"
#include "slate.h"

// Make sure top gpio bank enabled
static_assert(PICO_RP2350A == 0,
              "PICO_RP2350A must be defined to 0 for PICUBED builds.");

/**
 * One slate to rule them all!
 */
slate_t slate;

int main()
{
    stdio_init_all();

    sleep_ms(5000);

    LOG_INFO("[main] Slate takes up %d bytes!", sizeof(slate));
    // sched_init(&slate);

    /*
     * Run global initialization
     */

    LOG_INFO("[main] Running global init...");
    init(&slate);

    /*
     * Initialize the state machine
     */
    sleep_ms(1000);

    LOG_INFO("[main] Slate takes up %d bytes!", sizeof(slate));
    sched_init(&slate);

    /*
     * Run the state machine for all of time
     */
    LOG_INFO("[main] Initialization sequence complete - beginning main loop!");

    while (1)
    {
        test_sun_pyramid_reading(&slate);
        sleep_ms(1000);
    }

    // We should not be here -> very bad
    ERROR("Reached the end of the code! This should not happen!");
}