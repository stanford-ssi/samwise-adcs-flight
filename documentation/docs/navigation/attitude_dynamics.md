# Attitude Dynamics

Quaternion kinematics propagation using IMU angular velocity data.

**Location:** `src/gnc/attitude_dynamics.{h,cpp}`

## Functions
```cpp
void propagate_attitude(slate_t *slate);      // Propagate quaternion using IMU data
void test_propagate_attitude(slate_t *slate); // Test function
```

## Implementation
**Quaternion kinematics:** `q̇ = 0.5 * q ⊗ [0; ω]`
**Integration:** Euler method with quaternion normalization
**Input:** `slate->angular_velocity` from IMU
**Output:** Updated `slate->attitude_quaternion`

Used by attitude filter for state propagation between measurement updates.