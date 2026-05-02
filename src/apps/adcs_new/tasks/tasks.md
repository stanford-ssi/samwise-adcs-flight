# ADCS Subsystem Tasks
We have a set of 14 tasks. 4 sensing, 3 filtering, 3 control,
2 telemetry tasks, 1 watchdog, and 1 state machine task.

## Sensing
GPS Task

This task responds to uart interrupts to handle GPS data.
This then should store all data related to GPS into a single
struct within the slate.

IMU Task

This task should read the IMU for angular velocity and for
linear acceleration. Angular velocity is super critical for
attitude filtering and so this task is extremely important.

Magnetometer Task

This task should read the magnetometer to get both raw data
and calibrated data. This is also extremely critical as it
provides a (hopefully) unbiased reference for attitude filtering.

Sun Sensor Task

This task reads the rp2350 ADC and external ADC to get TIA current.
This is then converted to a solar intensity and can be used 
to generate a sun vector reading. This will be the second of our
two reference vectors.

## Filtering
Fuse Sensor

Upon receiving a sensor reading the system can update the Kalman
filter. This does
 1. Compute Sensitivity Matrix
 2. Compute Kalman Gain
 3. Update Covariance
 4. Compute Residual
 5. Update error state

Reset Errors

This task adds the errors into the cumulative state, and it resets
the error estimates to zero. This should run after a few sensor
readings have been fused.

Propagate State

This uses the IMU reading to propagate the state. It also updates
Covariances based on the Kalman filter dynamic model. Propagation
is done using numerical integration.

## Control
Torque Determination

This task runs based on sensor feedback and determines a torque
to get to the desired attitude.

Reaction Wheels

This task will communicate with the Reaction Wheel board to get
reaction wheel status and it will send out calculated torques.

Magnetorquers

This task uses the calculated torque to control currents for the
magnetorquers.

## Telemetry
PiCubed Send

This task sends data to picubed.

PiCubed receive

This task reads and responds to data sent by picubed.

