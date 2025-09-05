# Getting Started

<div align="center">
  <img src="img/sats-gnc-banner.png" alt="GNC Banner" width="400">
</div>

Quick guide to start developing ADCS flight software.

## ADCS Basics

**Attitude Determination and Control System** - determines where the satellite points and controls its orientation.

**Key Components:**
- **Sensors**: Magnetometer, IMU, Sun sensors, GPS
- **Actuators**: Magnetorquers, Reaction wheels

**Why State Machines?**
Different operational modes for different circumstances:
- **Detumbling** - Stop spinning after rocket deployment
- **Attitude Determination (Sensors)** - Do nothing but figure out where we're pointing
- **Nadir Pointing** - Point cameras down towards the earth
- **Slewing** - Controlled attitude changes
- **Safe** - Low power fault recovery

## Three-Pillar Architecture

### 🔧 Drivers
Physical hardware interface layer.
- Converts hardware signals ↔ software data
- Examples: `magnetometer.cpp`, `magnetorquer.cpp`

### 🧮 GNC Algorithms  
Mathematical control theory implementation.
- Examples: `bdot.cpp` (detumbling), `attitude_filter.cpp` (estimation)

### ⚙️ Software Tasks
Task scheduling and state management.
- Examples: `scheduler.cpp`, `sensors_task.cpp`, `detumble_state.cpp`

## Development Setup

**Prerequisites:** Mac/Linux only (Windows users: Linux VM)

```bash
# Clone and setup
git clone <repo-url>
cd samwise-adcs-flight
source configure.sh

# Build
mkdir build && cd build
cmake .. && make -j8

# Flash to Raspberry Pi Pico
picotool load samwise-adcs.uf2 -f
# Or drag .uf2 file to RP2350B drive
```

**Tools:**
- VS Code with C/C++ extensions
- Serial monitor for debug output
- Git for version control

## Key Files

- `scheduler/scheduler.cpp` - System heart
- `states/detumble_state.cpp` - Main operational state
- `tasks/sensors_task.cpp` - Sensor data collection
- `gnc/bdot.cpp` - Core control algorithm
- `constants.h` - Tunable parameters
- `slate.h` - Central data structure

## The "Slate" Data Hub

All system data lives in `slate_t`:
```cpp
slate_t slate = {
    .b_field_local = {1e-6, 2e-6, -5e-5},  // Magnetic field [T]
    .w_mag = 0.05,                          // Angular velocity [rad/s]
    .current_state = &detumble_state,       // Current state
    .magmeter_alive = true,                 // Sensor status
};
```

## Testing

Use serial monitor to watch system:
```bash
ls /dev/tty.*
tio /dev/tty.usbmodem[XXX]
```

Look for `LOG_INFO()` and `LOG_DEBUG()` messages.

## Next Steps

1. Understand scheduler and state machine
2. Work on driver development
3. Implement GNC algorithms
4. Focus on robust error handling
5. Learn control theory behind B-dot and attitude determination

**Safety first, then functionality.** ADCS can destroy the mission if it malfunctions.

<div align="center">
  <img src="img/samwise-render-tundra.png" alt="Mission Success" width="400">
</div>

Welcome to the team! 🛰️