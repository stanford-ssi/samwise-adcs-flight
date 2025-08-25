/**
 * @author  Lundeen Cahilly
 * @date    2025-08--02
 *
 * blink an led. this was harder than it should have been. godforsaken neopixel.
 */

#include "../drivers/neopixel.h"
#include "macros.h"
#include "pico/stdlib.h"

void blink(void)
{
    while (1)
    {
        // Set neopixel to white
        LOG_INFO("[blink] Wax on");
        neopixel_set_color_rgb(255, 255, 255);

        // Wait for a second
        sleep_ms(1000);

        // Turn off neopixel
        LOG_INFO("[blink] Wax off");
        neopixel_off();

        // Wait for another second
        sleep_ms(1000);
    }
}
