/**
 * @author Lundeen Cahilly
 *
 * This is the brick file, where I announce that I am a brick!
 */

#include "pico/stdlib.h"
#include "macros.h"

int main()
{
    stdio_init_all();

    sleep_ms(5000);

    /*
     * Run global initialization
     */
    LOG_INFO("[main] Brick starting...");
    while (1)
    {
        LOG_INFO("[main] I am Brick!");
        sleep_ms(1000);
    }
}
