---
title: "System Architecture"
owner: "@flight-software-team"
status: "active"
order: 2
parent: "Core Systems"
---

# System Architecture

The SAMWISE ADCS flight software is built around a real-time task scheduler and state machine architecture designed for reliability and deterministic behavior in space environments.

## Core Components

### Task Scheduler
- **Fixed-priority preemptive scheduling** for deterministic behavior
- **Static memory allocation** - no heap allocation during flight
- **Configurable task periods** from milliseconds to minutes
- **Watchdog integration** for fault detection

### State Machine
The satellite operates in discrete states with well-defined transitions:

```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│    INIT     │───▶│  DETUMBLE    │───▶│   STABLE    │
└─────────────┘    └──────────────┘    └─────────────┘
       │                   │                   │
       │                   ▼                   ▼
       │            ┌──────────────┐    ┌─────────────┐
       └───────────▶│  SAFE_MODE   │◀───│   SLEWING   │
                    └──────────────┘    └─────────────┘
```

### Hardware Abstraction Layer (HAL)
- **Sensor drivers**: IMU, magnetometer, sun sensors
- **Actuator control**: Reaction wheels, magnetorquers
- **Communication interfaces**: UART, I2C, SPI
- **Power management**: Current monitoring, thermal control

## Memory Architecture

### Static Allocation
All memory is allocated at compile time using a "slate" structure:

```cpp
struct SystemSlate {
    SensorData sensors;
    ControlState control;
    NavigationData nav;
    TelemetryBuffer telemetry;
    
    // Task-specific memory pools
    uint8_t task_stacks[NUM_TASKS][TASK_STACK_SIZE];
    MessageQueue message_queues[NUM_QUEUES];
};

static SystemSlate g_slate;
```

### Memory Pools
- **Sensor data**: Ring buffers for IMU, magnetometer readings
- **Control history**: State estimation and control output logs
- **Telemetry**: Downlink message queues
- **Debug logs**: Circular buffer for diagnostics

## Inter-Task Communication

### Message Passing
Tasks communicate through typed message queues:

```cpp
enum MessageType {
    MSG_SENSOR_DATA,
    MSG_CONTROL_CMD,
    MSG_STATE_CHANGE,
    MSG_TELEMETRY
};

struct Message {
    MessageType type;
    uint32_t timestamp;
    union {
        SensorReading sensor;
        ControlCommand control;
        StateTransition state;
        TelemetryPacket telemetry;
    } data;
};
```

### Shared State
Critical system state is protected by mutexes:
- **Attitude estimate** (quaternion, angular velocity)
- **Orbit state** (position, velocity from GPS)
- **Control mode** and target attitude
- **System health** status

## Real-Time Characteristics

### Timing Requirements
- **Sensor sampling**: 100 Hz (IMU), 10 Hz (magnetometer)
- **Control loop**: 50 Hz attitude control
- **Telemetry**: 1 Hz beacon, on-demand high-rate
- **State estimation**: 20 Hz Kalman filter update

### Interrupt Priority Levels
1. **Hardware watchdog** (highest priority)
2. **Critical sensor interrupts** (IMU data ready)
3. **Timer interrupts** (task scheduler)
4. **Communication interrupts** (UART, I2C)
5. **General GPIO** (lowest priority)

## Error Handling

### Fault Detection
- **Sensor validation**: Range checks, consistency tests
- **Control saturation**: Actuator limit monitoring  
- **Communication timeouts**: Heartbeat mechanisms
- **Stack overflow**: Guard patterns in task stacks

### Recovery Strategies
- **Sensor failure**: Graceful degradation to backup sensors
- **Control divergence**: Automatic safe mode transition
- **Communication loss**: Autonomous beacon mode
- **Persistent faults**: System reset with fault logging

## Development Environment

### Build System
- **CMake** for cross-platform builds
- **ARM GCC toolchain** for RP2350 target
- **Static analysis** with `cppcheck` and `clang-tidy`
- **Unit testing** with embedded test framework

### Debugging Tools
- **Serial console** for runtime diagnostics
- **Hardware debugger** (SWD/JTAG) support
- **Flight data recorder** for post-mission analysis
- **Simulation environment** for ground testing

This architecture provides a robust foundation for autonomous satellite operations while maintaining the determinism and reliability required for space missions.