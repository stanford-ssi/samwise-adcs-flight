/**
 * @author Lundeen Cahilly
 * @date 2025-08-02
 *
 * Driver for neopixel led
 */

#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize neopixel driver
 * @return true if initialization successful, false otherwise
 */
bool neopixel_init(void);

/**
 * Set neopixel color using RGB values (0-255)
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @return true if color was set successfully, false otherwise
 */
bool neopixel_set_color_rgb(uint8_t r, uint8_t g, uint8_t b);

/**
 * Turn off neopixel (set to black)
 * @return true if successful, false otherwise
 */
bool neopixel_off(void);

/**
 * Check if neopixel driver is initialized
 * @return true if initialized, false otherwise
 */
bool neopixel_is_initialized(void);

#endif // NEOPIXEL_H