# ADM1176 Power Monitor

Power monitor IC that measures ADCS voltage, current, and power consumption via I2C.

**Location:** `src/drivers/adm1176.{h,cpp}`

## Key Constants
- I2C Address: `0x4A`
- Voltage scale: `26.35f / 4096.0f`
- Current scale: `0.10584f / 4096.0f` 
- Sense resistor: `0.0207Ω`
- Timeout: `100ms`

## Functions
```cpp
bool adm_init(void);                           // Initialize driver
bool adm_power_on(void);                      // Power on ADM1176
bool adm_power_off(void);                     // Power off ADM1176
bool adm_get_voltage(float *voltage_out);     // Read voltage
bool adm_get_current(float *current_out);     // Read current
bool adm_get_power(slate_t *slate);           // Read all, store in slate
```

## Implementation
- **Power on sequence:** Send `0x83, 0x00` command, configure continuous mode
- **Reading:** 3-byte I2C read, extract 12-bit voltage/current values
- **Conversion:** Apply scaling factors and sense resistor correction
- **Output:** Updates `slate->adcs_voltage`, `slate->adcs_current`, `slate->adcs_power`

Used for power monitoring and stored in telemetry via the slate structure.