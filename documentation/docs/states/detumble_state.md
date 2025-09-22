# Detumble State

Magnetic damping control state to reduce angular rates using B-dot algorithm.

**Location:** `src/states/detumble_state.{h,cpp}`

## Active Tasks
```cpp
.task_list = {&sensors_task, &telemetry_task, &bdot_task, &watchdog_task}
```

## Functions  
```cpp
sched_state_t *detumble_get_next_state(slate_t *slate);  // State transition logic
```

## Transitions
**To Slewing:** `slate->w_mag < W_EXIT_DETUMBLE_THRESHOLD` (low angular rates)
**To Cool Down:** `slate->w_mag > W_COOL_DOWN_ENTER_THRESHOLD` (excessive rates)

## Implementation
- **Visual:** Blue neopixel LED `neopixel_set_color_rgb(0, 0, 255)`
- **Control:** B-dot task uses magnetometer data to command magnetorquers
- **Monitoring:** IMU provides angular velocity magnitude for transition decisions

First active control state after initialization, reduces post-deployment tumbling rates.