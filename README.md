# SAMWISE ADCS Flight Software
Flight Software for the SAMWISE ADCS board

# Directory Structure
* **Top Level**: Main file and general utilities (type definitions, macros, linear algebra library, etc.)
* `drivers`: Functions for interacting directly with hardware (IMU, GPS, magmetometers, etc.)
* `gnc`: Math routines for control functions (attitute estimation, control, etc.)
* `scheduler`: Core of the software called directly by `main` - handles state transitions and dispatches tasks
### State Machine:
* `states`: Contains `.cpp` and `.h` files for each state that the board can be in. Each state contains a list of tasks to dispatch and a transition function to determine the next state.
* `tasks`: Contains `.cpp` and `.h` files for each task - tasks are dispatched by the scheduler depending on the current state

## Build Products
For the latest build products, check `actions -> C build -> samwise-adcs-products.zip`.

## Install Instructions
Follow [the flight software onboarding doc](docs/ONBOARDING.md) for development environment setup.

Run `source configure.sh` for initial repository setup.

**Note: Only Mac and Linux are supported. If you are on Windows, create a Linux VM**

## Building Instructions
Enter the `build` folder:
```
cd build
```

Run `cmake` (Note: this is usually only necessary if you have added, moved, or deleted files):
```
cmake ..
```

Make the project:
```
make -j8
```

**Only on Mac:**
```
cp samwise-adcs.uf2 /Volumes/RP2350 
```

Follow instructions [here](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html#blink-an-led) for how to drag the `.uf2` file onto your device.

### Useful Links
* [Picubed flight software repo](https://github.com/stanford-ssi/samwise-flight-software)
* [ADCS sims repo](https://github.com/stanford-ssi/samwise-adcs-sims)


## Things to implement
* Bdot for detumbling
* Attitude determination
    * Determine triad from magnetic field and sun vector
        * Magnetic field model
        * Sun vector
    * Propagator
    * Kalman filter
* Control
    * Total torque control
    * Allocation of control to wheels/magnetorquers
* Deasturation of reaction wheels