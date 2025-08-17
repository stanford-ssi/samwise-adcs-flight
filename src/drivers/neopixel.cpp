/**
 * @author Lundeen Cahilly
 * @date 2025-08-02
 *
 * Driver for neopixel led
 */

#include "neopixel.h"
#include "external/ws2812.pio.h"
#include "hardware/gpio.h"
#include "macros.h"
#include "pico/stdlib.h"
#include "pins.h"
#include <stdint.h>

// Neopixel configuration
#define NEOPIXEL_FREQ 800000.0f
#define COLOR_DEPTH_SHIFT 3 // Maps 8-bit (0-255) to 5-bit (0-31)

// Static state
static PIO neopixel_pio = pio0;
static uint neopixel_sm = 0;
static bool is_initialized = false;

/**
 * Convert RGB values to GRB format for WS2812
 */
static inline uint32_t rgb_to_grb_u32(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (uint32_t)(b);
}

/**
 * Send pixel data to neopixel
 */
static inline void send_pixel(uint32_t pixel_grb)
{
    pio_sm_put_blocking(neopixel_pio, neopixel_sm, pixel_grb << 8);
}

/**
 * Initialize neopixel driver
 * @return true if initialization successful, false otherwise
 */
bool neopixel_init(void)
{
#ifndef PICO
    // Initialize GPIO pin
    gpio_init(SAMWISE_ADCS_NEOPIXEL);
    gpio_set_dir(SAMWISE_ADCS_NEOPIXEL, GPIO_OUT);

    // Add PIO program
    uint offset = pio_add_program(neopixel_pio, &ws2812_program);
    if (offset == -1)
    {
        LOG_ERROR("[neopixel] Failed to add WS2812 PIO program");
        return false;
    }

    // Initialize PIO state machine
    ws2812_program_init(neopixel_pio, neopixel_sm, offset,
                        SAMWISE_ADCS_NEOPIXEL, NEOPIXEL_FREQ, true);

    is_initialized = true;
    LOG_INFO("[neopixel] Neopixel initialized on pin %d at %.0f Hz",
             SAMWISE_ADCS_NEOPIXEL, NEOPIXEL_FREQ);

    return true;
#else
    LOG_INFO("[neopixel] Neopixel driver initialized (simulation mode - PICO "
             "build)");
    is_initialized = true;
    return true;
#endif
}

/**
 * Set neopixel color using RGB values (0-255)
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @return true if color was set successfully, false otherwise
 */
bool neopixel_set_color_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (!is_initialized)
    {
        LOG_ERROR("[neopixel] Neopixel not initialized");
        return false;
    }

#ifndef PICO
    // Reduce color depth from 8-bit to 5-bit
    r = r >> COLOR_DEPTH_SHIFT;
    g = g >> COLOR_DEPTH_SHIFT;
    b = b >> COLOR_DEPTH_SHIFT;

    uint32_t pixel_data = rgb_to_grb_u32(r, g, b);
    send_pixel(pixel_data);

    // LOG_DEBUG("[neopixel] Neopixel color set: R=%d G=%d B=%d", r, g, b);
#else
    LOG_DEBUG("[neopixel] Neopixel color set (simulation): R=%d G=%d B=%d", r,
              g, b);
#endif

    return true;
}

/**
 * Turn off neopixel (set to black)
 * @return true if successful, false otherwise
 */
bool neopixel_off(void)
{
    return neopixel_set_color_rgb(0, 0, 0);
}

/**
 * Check if neopixel driver is initialized
 * @return true if initialized, false otherwise
 */
bool neopixel_is_initialized(void)
{
    return is_initialized;
}