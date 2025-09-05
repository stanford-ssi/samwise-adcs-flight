# Neopixel Status LED

RGB LED for system status indication via WS2812 protocol.

**Location:** `src/drivers/neopixel.{h,cpp}`

## Functions
```cpp
bool neopixel_init(void);                              // Initialize neopixel driver
bool neopixel_set_color_rgb(uint8_t r, uint8_t g, uint8_t b); // Set RGB color (0-255)
bool neopixel_off(void);                              // Turn off LED
bool neopixel_is_initialized(void);                   // Check if initialized
```

## Implementation
- **Protocol:** WS2812 timing-critical bit-banged interface
- **Colors:** RGB values 0-255 each
- **Usage:** System state indication (e.g. blue for detumble state)

Used by state machine to visually indicate current operational mode.