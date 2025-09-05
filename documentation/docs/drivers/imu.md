# BMI270 IMU

6-axis IMU (gyro + accel) for angular velocity measurements. Uses Bosch BMI270 library.

**Location:** `src/drivers/imu.{h,cpp}` + `src/drivers/external/bmi2*`

## Key Constants
- Chip ID: `0x24` (BMI_EXPECTED_CHIP_ID)
- I2C Address: `BMI2_I2C_PRIM_ADDR`

## Functions
```cpp
void imu_power_disable();              // Turn off IMU power
void imu_power_enable();               // Turn on IMU power  
bool imu_init();                       // Initialize IMU with BMI2 library
bool imu_get_rotation(float3 *w_out);  // Read angular velocity [rad/s]
```

## Implementation
- **Power control:** GPIO-controlled power rail enable/disable
- **Initialization:** Power on, verify chip ID (0x24), init BMI2 library
- **Reading:** Get gyro data through BMI2 library, convert to rad/s
- **Output:** 3-axis angular velocity in body frame

Used by attitude filter for high-rate angular velocity measurements.