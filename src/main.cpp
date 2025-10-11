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

#ifdef SIMULATION
    LOG_INFO("========================================");
    LOG_INFO("  SIMULATION MODE - Hardware-in-Loop");
    LOG_INFO("========================================");
    LOG_INFO("[main] Build mode: SIMULATION");
    LOG_INFO("[main] Sensor data will be received via USB");
    LOG_INFO("[main] Actuator commands will be sent via USB");
#elif defined(TEST)
    LOG_INFO("========================================");
    LOG_INFO("  TEST MODE - Hardware Testing");
    LOG_INFO("========================================");
    LOG_INFO("[main] Build mode: TEST");
#else
    LOG_INFO("========================================");
    LOG_INFO("  FLIGHT MODE - Production");
    LOG_INFO("========================================");
    LOG_INFO("[main] Build mode: FLIGHT");
#endif

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
        sched_dispatch(&slate);
    }

    // We should not be here -> very bad
    ERROR("Reached the end of the code! This should not happen!");
}