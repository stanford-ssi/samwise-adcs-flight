# Attitude Filter (Extended Kalman Filter)

7-state EKF for attitude estimation using IMU, magnetometer, and sun sensor data.

**Location:** `src/gnc/attitude_filter.{h,cpp}`

## State Vector
```cpp
x = [q_w, q_x, q_y, q_z, ω_x, ω_y, ω_z]ᵀ  // Quaternion + angular velocity
```

## Functions
```cpp
void attitude_filter_init(slate_t *slate);                           // Initialize filter
void attitude_filter_propagate(slate_t *slate, float dt);           // Predict step with IMU
void attitude_filter_update(slate_t *slate, quaternion q_meas_eci_to_principal); // Update step
void test_attitude_filter(slate_t *slate);                          // Test function
```

## Noise Matrices
**Process noise (Q):** Diagonal 7x7, quaternion = 0.01, angular velocity = 0.02
**Measurement noise (R):** 4x4 quaternion covariance, diagonal = 0.04, off-diagonal = -0.01

## Implementation
- **Predict:** Propagate quaternion kinematics `q̇ = 0.5 * q ⊗ [0; ω]`, update covariance
- **Update:** Fuse quaternion measurements from sun/mag vector matching
- **Output:** `slate->attitude_quaternion` and `slate->angular_velocity`

Used for optimal attitude estimation by fusing high-rate IMU with periodic attitude references.