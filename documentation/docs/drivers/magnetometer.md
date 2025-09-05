# RM3100 Magnetometer

3-axis magnetometer for magnetic field measurement via SPI. Used for B-dot control and attitude determination.

**Location:** `src/drivers/magnetometer.{h,cpp}`

## Key Constants
- Revision ID: `0x22`
- Cycle Count: `200` (resolution setting)
- Sensitivity: `75.0f` LSB/µT
- SPI Frequency: `1MHz`
- Sample Rate: `75Hz`

## Functions
```cpp
rm3100_error_t rm3100_init(void);                        // Initialize magnetometer
rm3100_error_t rm3100_get_reading(float3 *mag_field);    // Read 3-axis field in µT
```

## Implementation
- **Initialization:** Verify chip ID (0x22), set cycle count to 200, configure 75Hz continuous mode
- **Reading:** Check status register, read 24-bit signed values for X/Y/Z, convert to µT
- **SPI:** 1MHz, Mode 0, GPIO chip select, 24-bit data format
- **Output:** 3-axis magnetic field vector in microTesla

Used by B-dot algorithm for detumbling control and attitude filter for reference vector.