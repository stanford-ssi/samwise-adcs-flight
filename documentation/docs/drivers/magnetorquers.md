# Magnetorquer PWM Control

3-axis magnetic torque coils controlled via PWM for spacecraft attitude control.

**Location:** `src/drivers/magnetorquer.{h,cpp}` + `src/drivers/pwm_driver.c`

## Key Constants
- PWM range: `-128` to `+128`
- Default max current: `300mA`
- Error types: `PWM_OK`, `PWM_ERROR_OUT_OF_RANGE`, `PWM_ERROR_CURRENT_EXCEEDED`, `PWM_ERROR_INVALID_PARAM`

## Functions
```cpp
void init_magnetorquer_pwm(void);                                    // Initialize PWM system
pwm_error_t do_magnetorquer_pwm(int8_t xdn, int8_t ydn, int8_t zdn,  // Set duty cycles
                                int max_current);
void stop_magnetorquer_pwm(void);                                   // Emergency stop
```

## Implementation
- **Physics:** Magnetic dipole `m` creates torque `τ = m × B` with Earth's field
- **Control:** PWM duty cycle -128 to +128 maps to current direction/magnitude
- **Safety:** Current limiting prevents coil overheating
- **Output:** 3-axis magnetic moments for spacecraft torque generation

Used by B-dot algorithm for magnetic damping control during detumbling.