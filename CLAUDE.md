# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Common Development Commands

### Environment Setup
- Run `source configure.sh` for initial repository setup (git submodules, pico-sdk setup, build directory)
- Install Arm GNU Toolchain: `brew install --cask gcc-arm-embedded` (Mac)

### Building
```bash
cd build
cmake ..          # Only needed if files added/moved/deleted
make -j8          # Build the project
```

### Deployment
```bash
picotool load samwise-adcs.uf2 -f
```

### Code Formatting
- `./format_all.sh` - Formats all source files using clang-format
- Pre-commit hook automatically runs formatting

## Architecture Overview

### Core Architecture Pattern
The system uses a **blackboard pattern** centered around a single global `slate_t` struct that contains all satellite state. This struct is passed to all functions and tasks, providing a centralized data store.

### State Machine Design
- **Scheduler** (`src/scheduler/`): Core dispatcher that manages state transitions and task execution
- **States** (`src/states/`): Define satellite operational modes (init, detumble, slewing, cool_down, test)
- **Tasks** (`src/tasks/`): Specific operations dispatched by states (sensors, bdot, telemetry, test)
- Each state contains a list of tasks to execute and a transition function for the next state
- Tasks have configurable dispatch periods and are only run when their time interval has elapsed

### Key Components
- **slate_t** (`src/slate.h`): Central data structure containing sensor data, actuator requests, GNC state, and state machine info
- **GNC** (`src/gnc/`): Guidance, Navigation & Control algorithms including attitude estimation, control laws, and world models
- **Drivers** (`src/drivers/`): Hardware interfaces for IMU, GPS, magnetometer, magnetorquers, and PiCubed UART
- **Linear Algebra** (`src/linalg.h`): Mathematical utilities for vector/matrix operations using linalg library

### Hardware Platform
- **Target**: Raspberry Pi Pico (RP2350b) microcontroller
- **Board**: Custom ADCS board (`boards/adcs.h`)
- **SDK**: Pico SDK with CMake build system
- **Communication**: USB stdio enabled, UART disabled by default

### Development Workflow
1. Main loop runs state machine dispatcher (`sched_dispatch`)
2. Scheduler checks current state's task list and executes ready tasks
3. State transition function determines next state based on current slate
4. Tasks update slate with sensor readings, control outputs, or telemetry
5. Actuator drivers read requested values from slate

### Testing
- Test functions in `src/tests/` for hardware validation
- Main file currently configured for magnetic field model testing
- No standard test framework - uses custom test functions