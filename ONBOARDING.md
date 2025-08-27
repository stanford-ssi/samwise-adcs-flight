# SAMWISE ADCS Flight Software Onboarding Guide ⚡🏴‍☠️

<div align="center">
  <img src="docs/images/sats-gnc-banner.png" alt="GNC Pirate Flag" width="400">
</div>

Welcome to the SSI Satellites GNC team! This guide will get you up to speed on our flight software system, whether you're a frosh, returning student, or an aero/astro grad student. 🛰️

If you've never seen anything about a satellite before, please watch [Mark Rober's selfie satellite video](https://www.youtube.com/watch?v=6KcV1C1Ui5s&ab_channel=MarkRober). It gives an incredible overview of a full satellite system, and the selfie mission is oddly similar to ours. 

## Table of Contents

1. [Understanding ADCS Basics](#understanding-adcs-basics)
2. [The Three-Pillar Architecture](#the-three-pillar-architecture)
3. [Development Environment Setup](#development-environment-setup)
4. [Working with the Codebase](#working-with-the-codebase)
5. [Claude Code Workflows](#claude-code-workflows)
6. [Common Development Tasks](#common-development-tasks)
7. [Testing and Validation](#testing-and-validation)

## Understanding ADCS Basics

### What is ADCS?
**Attitude Determination and Control System** - the subsystem that determines where the satellite is pointing and controls its orientation in space.

<div align="center">
  <img src="docs/images/adcs-components-diagram.png" alt="ADCS Components" width="700">
  <p><em>ADCS hardware components and their functions</em></p>
</div>

### Key Components
- **Sensors**: Tell us our current attitude and angular velocity
  - **Magnetometer**: Measure Earth's magnetic field
  - **IMU** (Inertial Measurement Unit): Measures angular velocity and acceleration
  - **Sun sensors**: Detect sunlight direction  
  - **GPS**: Provides position data
- **Actuators**: Spin the satellite
  - **Magnetorquers**: Create magnetic fields to interact with Earth's field
  - **Reaction wheels**: Spinning wheels that transfer angular momentum

<div align="center">
  <img src="docs/images/bdot-control-diagram.png" alt="B-dot Control" width="600">
  <p><em>B-dot control: Using Earth's magnetic field for detumbling</em></p>
</div>

### Why State Machines?
Spacecraft operate in different modes depending on circumstances:
- **Detumbling**: Mode to stop the satellite from spinning upon ejection from the rocket
- **Safe**: Safe mode to reduce power consumption
- [TODO: more to be added]

Each mode needs different sensors, algorithms, and safety checks.

## The Three-Pillar Architecture ⚡

<div align="center">
  <img src="docs/images/three-pillars-pirate.png" alt="Three Pillars of Pirate ADCS" width="700">
  <p><em>The ADCS flight software is built on three foundational pillars</em></p>
</div>

Our flight software follows a clean architectural pattern with three distinct layers that work together seamlessly:

### 🔧 Pillar 1: Drivers
**The Physical Interface Layer**

<div align="center">
  <img src="docs/images/hardware-drivers-flow.png" alt="Hardware Drivers" width="600">
  <p><em>Driver layer: Converting between hardware signals and software data</em></p>
</div>

**What it does:** Translates between raw hardware signals and raw data that we can use in later algorithms.

**Notable Files:**
- `drivers/magnetometer.cpp` - Reads magnetic field measurements
- `drivers/magnetorquer.cpp` - Commands magnetic torque generation
- etc.

This is the embedded systems interface - I2C, SPI, UART, PWM, and register manipulation. 

### 🧮 Pillar 2: GNC Algorithms 🏴‍☠️
**The Mathematical Brain**

<div align="center">
  <img src="docs/images/gnc-logo-pirate.png" alt="GNC Pirate Logo" width="400">
  <p><em>GNC layer: Where control theory meets spacecraft dynamics</em></p>
</div>

**What it does:** Implements the control theory and mathematical models that make ADCS work.

**Notable Files:**
- `gnc/bdot.cpp` - Magnetic detumbling control law
- `gnc/attitude_filter.cpp` - Kalman filtering for attitude estimation. Takes in raw data and spits out an estimation of where we're pointing & angular velocity

This is where applied mathematics meets spacecraft control - linear algebra, filtering, spacecraft dynamics, and control theory.

### ⚙️ Pillar 3: Software Tasks
**The Orchestration Layer**

<div align="center">
  <img src="docs/images/task-scheduling-flow.png" alt="Task Scheduling" width="600">
  <p><em>Scheduling layer: Coordinating when and how everything happens</em></p>
</div>

**What it does:** Manages when tasks run, coordinates state transitions, and ensures system safety.

**Key Files:**
- `scheduler/scheduler.cpp` - The main orchestrator
- `tasks/sensors_task.cpp` - Coordinates all sensor readings
- `tasks/bdot_task.cpp` - Executes detumbling control
- `states/detumble_state.cpp` - Defines detumbling behavior

This is systems programming - state machines, real-time scheduling, and task coordination that ensures control algorithms run at the right times with the right priorities.

### How the Pillars Work Together

```
Sensor Hardware → Drivers → Data → GNC Algorithms → Control Commands → Drivers → Actuator Hardware
```

### The Task-Based State Machine

Our system uses a **cooperative multitasking** approach:

```
Main Loop → Scheduler → Current State → Tasks
                ↓
         State Transition Logic
```

#### How It Works:

1. **Scheduler (`scheduler.cpp`)** calls the current state
2. **State** (`states/*.cpp`) runs its assigned tasks if they're due
3. **Tasks** (`tasks/*.cpp`) do the actual work (read sensors, run control, etc.)
4. **State transition function** decides if we should change states

#### Example: Detumble State Flow

```cpp
detumble_state runs:
├── sensors_task (every [X]ms) - Read magnetometer, IMU
├── bdot_task (every [X]ms) - Compute magnetic control
├── telemetry_task (every [X]ms) - Send data to spacecraft
└── watchdog_task (every [X]ms) - Feed watchdog timer

Then checks: if (angular_velocity < 1°/s) → switch to [TODO: state]]
```

### Data Flow: The "Slate"

All sensor data and state information lives in a structure called `slate_t`:

```cpp
slate_t slate = {
    .b_field_local = {1e-6, 2e-6, -5e-5},  // Magnetic field (Tesla)
    .w_mag = 0.05,                          // Angular velocity magnitude (rad/s)
    .current_state = &detumble_state,       // Which state we're in
    .magmeter_alive = true,                 // Is magnetometer working?
    // ... more data
};
```

Tasks read from and write to the slate, making it the central data hub. Think of it like every file reads from the same whiteboard!

## Development Environment Setup

<div align="center">
  <img src="docs/images/development-setup.png" alt="Development Environment" width="700">
  <p><em>Your development environment: VS Code + hardware + serial terminal</em></p>
</div>

### Prerequisites
- **Mac or Linux** (Windows users: create a Linux VM)
- Git
- CMake
- ARM GCC toolchain
- Python 3 (for scripts)
- VS Code with C/C++ extensions

### Initial Setup

1. **Clone and configure:**
   ```bash
   git clone <repository-url>
   cd samwise-adcs-flight
   source configure.sh  # Sets up build environment
   ```

2. **Build the project:**
   ```bash
   cd build
   cmake ..
   make -j8
   ```

3. **Flash to hardware (Raspberry Pi Pico):**
   ```bash
   picotool load samwise-adcs.uf2 -f
   ```
   Or drag the .uf2 file to the RP2350B drive manually 

### Development Tools
- **VS Code** with C/C++ extensions
- **Serial monitor** for debug output
- **Git** for version control
- **Claude Code** for AI-assisted development (see workflows I've found helpful below)

## Working with the Codebase

### File Organization
```
src/
├── main.cpp              # Entry point
├── constants.h           # System constants and thresholds
├── slate.h              # Central data structure
├── scheduler/           # State machine core
├── states/             # State definitions
├── tasks/              # Task implementations  
├── drivers/            # Hardware interfaces
├── gnc/               # Control algorithms
└── tests/             # Hardware tests
```

### Key Files to Understand

1. **`scheduler/scheduler.cpp`**: The heart of the system
2. **`states/detumble_state.cpp`**: Most important operational state
3. **`tasks/sensors_task.cpp`**: How we read sensors
4. **`tasks/actuators_task.cpp`**: How we run actuators
5. **`gnc/bdot.cpp`**: Core detumbling algorithm
6. **`constants.h`**: Tunable parameters

### Coding Conventions
- **C++ style**: CamelCase for classes, snake_case for variables/functions
- **Units / Coordinates**: Always document units in comments (rad/s, Tesla, etc.) as well as coordinates. Especially for GNC tasks, this will make or break our code.

## Claude Code Workflows

<div align="center">
  <img src="docs/images/claude-code-workflow.png" alt="Claude Code Workflow" width="700">
  <p><em>Using Claude Code for efficient ADCS development</em></p>
</div>

We use Claude Code extensively for development. Here are proven workflows that work exceptionally well for embedded systems and spacecraft control:

### Driver Development Workflow

When creating a new sensor/actuator driver:

1. **Preparation Phase:**
   ```
   "Help me develop a driver for [sensor name]. First, let me provide you with:
   - The sensor datasheet (upload PDF)
   - An example of an existing driver for reference style
   - Communication protocol details (I2C/SPI/UART)
   
   Please familiarize yourself with these first, then I'll ask you to write the code."
   ```
   Note: a key part of this workflow is to also **read the datasheet** and **familiarize yourself with SPI/I2C/UART**. This will enable you to debug very fast and avoid pushing bugged code. Trust me on this one.

2. **Implementation Phase:**
   ```
   "Now create a simple driver for [sensor]. Follow these requirements:
   - Keep the code very simple and readable
   - Follow the same structure as [reference driver]  
   - Include error handling for communication failures
   - Add comprehensive comments explaining register operations"
   ```

3. **Integration Phase:**
   ```
   "Add this driver to the sensors_task and update the slate structure 
   to include the new sensor data. Make sure to handle initialization 
   failures gracefully."
   ```

### Algorithm Development Workflow

For control algorithms or GNC functions:

1. **Context Setup:**
   ```
   "I'm implementing [algorithm name] for satellite attitude control.
   Here's the mathematical background: [equations/references]
   Look at our existing bdot.cpp for style reference."
   ```

2. **Implementation:**
   ```
   "Create a simple, well-commented implementation. Focus on:
   - Clear variable names with units
   - Input validation and bounds checking
   - Numerical stability
   - Integration with our slate data structure"
   ```

### Debugging Workflow

When encountering issues:

```
"I'm seeing [specific error/behavior]. Here's the relevant code [paste code].
The sensor readings show [data]. Help me debug this step by step:
1. Check the logic flow
2. Verify data validation  
3. Look for edge cases
4. Suggest test cases"
```

### Best Practices with Claude Code

✅ **Do:**
- Provide complete context (datasheets, existing code examples)
- Ask for simple, readable implementations first
- Request specific error handling and validation
- Ask for unit tests and validation procedures

❌ **Don't:**
- Ask for complex features without providing context
- Skip the familiarization phase for new components
- Forget to specify coding style requirements
- Ignore integration with existing systems

## Common Development Tasks

### Adding a New Sensor

1. **Create driver** (`drivers/new_sensor.cpp`, `drivers/new_sensor.h`)
2. **Add to sensors_task** in `sensors_task.cpp`
3. **Update slate structure** in `slate.h`
4. **Add initialization** to `sensors_task_init()`
5. **Test with hardware** using test state

### Creating a New Control Algorithm

1. **Implement in `gnc/`** following existing patterns
2. **Create corresponding task** in `tasks/`
3. **Add task to appropriate states**
4. **Test in simulation** before hardware
5. **Validate with real sensors**

### Adding a New State

1. **Define state structure** in `states/new_state.cpp`
2. **Implement transition logic** 
3. **Add to scheduler's state list**
4. **Update transition logic** in related states
5. **Test state transitions thoroughly**

### Modifying Control Parameters

1. **Update constants** in `constants.h`
2. **Document the change** with units and rationale
3. **Test in simulation** first
4. **Validate on hardware** with conservative values
5. **Monitor telemetry** for unexpected behavior

## Testing and Validation

<div align="center">
  <img src="docs/images/testing-setup.jpg" alt="Testing Setup" width="600">
  <p><em>Hardware-in-the-loop testing with ADCS board and sensors</em></p>
</div>

### Hardware-in-the-Loop Testing

Use the test state for hardware validation:

```cpp
// In test_state.cpp
test_magnetometer();    // Verify sensor readings
test_magnetorquer();    // Check actuator response  
test_bdot_algorithm();  // Validate control loop
```

### Serial Debug Output

Monitor system behavior:
```bash
ls /dev/tty.*
tio /dev/tty.usbmodem[XXX] 
```

Look for:
- `LOG_INFO()` messages showing normal operation  
- `LOG_DEBUG()` messages indicating problems or verbose info

## Next Steps

### Getting Started
1. Start by understanding the scheduler and state machine
2. Work on driver development for new sensors or GNC algorithms
3. Implement software improvements and optimizations
4. Focus on robust error handling and edge cases
5. Understand the control theory behind B-dot and attitude determination
6. Work on attitude estimation and filtering
7. Optimize control parameters and stability

### Advanced Projects
- Implement full attitude determination (MRP EKF)
- Add nadir pointing control using reaction wheels
- Develop wheel desaturation algorithms
- Create hardware-in-the-loop simulation interface
- Optimize power consumption and computational efficiency

Remember: **Safety first, then functionality**. The ADCS system can destroy the mission if it malfunctions, so we prioritize robust, well-tested code over complex features.

<div align="center">
  <img src="docs/images/samwise-render-tundra.png" alt="Mission Success" width="500">
</div>

**Welcome to the team!** 🛰️