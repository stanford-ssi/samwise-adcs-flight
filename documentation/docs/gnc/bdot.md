# B-dot Detumbling Algorithm

Magnetic damping control to reduce spacecraft angular rates using magnetorquers and Earth's B-field derivative.

**Location:** `src/gnc/bdot.{h,cpp}`

## Key Constants
- Control gain: `bdot_k = 1e5f`

## Functions
```cpp
float3 bdot_compute_control_proportional(float3 dB, float dt);  // Proportional control
float3 bdot_compute_control_bang_bang(float3 dB, float dt);     // Bang-bang control
void test_bdot_control(slate_t *slate);                         // Test function
```

## Control Laws

**Proportional:**
```cpp
const float3 bdot = dB / dt;
const float3 moments_raw = -bdot_k * bdot;
return clamp_abs_to_one(moments_raw);  // Clamp to ±1
```

**Bang-bang:**
```cpp
const float3 bdot = dB / dt;
return float3(sign(-bdot.x), sign(-bdot.y), sign(-bdot.z));
```

## Implementation
- **Input:** Magnetic field change `dB` and time step `dt`
- **Output:** Normalized magnetorquer moments `[-1, +1]`
- **Physics:** `m = -k * dB/dt` opposes rotation by creating torque `τ = m × B`

Used in detumble state to reduce post-deployment angular rates before attitude control.