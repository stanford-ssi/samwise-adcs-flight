# ADS7830 Sun Sensor ADC

8-bit, 8-channel ADC for reading sun sensor photodiode voltages via I2C.

**Location:** `src/drivers/ads7830.{h,cpp}`

## Functions
```cpp
bool ads7830_init(void);                                        // Initialize ADC
bool ads7830_get_reading(uint8_t channel, uint8_t *value);     // Read single channel (0-255)
bool ads7830_get_voltage(uint8_t channel, float *voltage, float vref); // Convert to voltage
bool ads7830_read_all_channels(uint8_t values_out[8]);         // Read all 8 channels
bool ads7830_read_all_voltages(float voltages_out[8]);         // Read all as voltages
```

## Implementation
- **Resolution:** 8-bit (0-255 values)
- **Channels:** 8 independent analog inputs (0-7)
- **Conversion:** `voltage = (adc_value / 255.0f) * vref` where vref = 3.3V
- **Interface:** I2C communication
- **Output:** Raw ADC values or converted voltages

Used to read sun sensor photodiode voltages for sun vector calculation and attitude determination.