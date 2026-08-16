# SAMWISE ADCS pre-flight kit

This folder provides a separate USB test image and guided host tools for:

- live magnetometer, IMU, and sun-sensor visualization;
- magnetometer hard/soft-iron calibration;
- IMU gyro bias calibration and optional accelerometer six-face characterization;
- magnetorquer, sun-sensor, IMU, and magnetometer polarity checks.

The test image is **not flight firmware**. It starts with all magnetorquer outputs off and does not run flight control, state estimation, telemetry, or B-dot. Re-flash `samwise-adcs-rewrite.uf2` after completing the checklist.

Print or copy [`CHECKLIST.md`](CHECKLIST.md) into the vehicle configuration record before starting.

## Safety gates

1. Keep the magnetorquer test for last. Every other workflow leaves the rods off.
2. A USB data connection does not prove that USB may safely supply the magnetorquer rail. Confirm the power path with the electrical owner before running `polarity-magnetorquer`. Use the normal current-limited bench/vehicle supply if required.
3. Remove reaction-wheel commands, motors, loose ferrous tools, permanent magnets, and high-current wiring from the magnetometer calibration area.
4. Support the vehicle before rotating it. Do not use flight connectors or harnesses as handles.
5. The magnetorquer command requires a literal `ARM`, expires after 30 seconds, energizes one axis only, clamps duty to 25% or less, clamps a pulse to 150–500 ms, and always stops before returning a result.
6. Close the tool normally or press Ctrl+C. The host sends `STOP`; the board also disarms automatically when its timer expires.

## 1. Build and flash the test image

From the repository root:

```sh
cmake -S . -B build
cmake --build build --target samwise-adcs-preflight -j
```

Flash `build/samwise-adcs-preflight.uf2`, not either flight UF2. With `picotool`:

```sh
picotool load build/samwise-adcs-preflight.uf2 -f
picotool reboot
```

If the board is in BOOTSEL mode, copy the UF2 to its mounted RP2350 volume instead. After reboot, connect the board's native USB port directly to the operator computer.

## 2. Create the visualization environment

macOS/Linux:

```sh
./pre_flight/setup.sh
PF_PY=pre_flight/.venv/bin/python
```

Windows PowerShell:

```powershell
py -m venv pre_flight/.venv
pre_flight/.venv/Scripts/python.exe -m pip install -r pre_flight/requirements.txt
$env:PF_PY = "pre_flight/.venv/Scripts/python.exe"
```

List ports and verify all four sensor interfaces:

```sh
$PF_PY pre_flight/run.py ports
$PF_PY pre_flight/run.py check --port auto
```

If auto-detection finds multiple devices, replace `auto` with the displayed `/dev/ttyACM…`, `/dev/tty.usbmodem…`, or `COM…` name.

Open the live dashboard at any time:

```sh
$PF_PY pre_flight/run.py monitor --port auto
```

All guided tests save timestamped JSON evidence and plots under `pre_flight/results/`. Nothing automatically edits flight constants.

## Recommended run order

### A. IMU calibration

For the deployable gyro zero-rate calibration:

```sh
$PF_PY pre_flight/run.py imu-calibrate --port auto
```

Place the complete flight assembly on a rigid, vibration-free surface. The reported `IMU_ZERO_READING_RPS` accounts for the offset already compiled into the test image. Copy the result into **both** `src/apps/adcs_app/params.h` and `src/apps/adcs_new/params.h`, rebuild the pre-flight image, and repeat once; the residual should be near zero.

For an additional six-face accelerometer characterization:

```sh
$PF_PY pre_flight/run.py imu-calibrate --six-face --port auto
```

The accelerometer report measures bias, per-axis scale, and cross-axis leakage. Those constants are diagnostic only: the current flight driver does not apply accelerometer calibration constants.

### B. IMU polarity

```sh
$PF_PY pre_flight/run.py polarity-imu --port auto
```

The tool performs two checks:

1. **Accelerometer:** place each requested BODY face upward. A stationary accelerometer should report positive specific force along the upward body axis and negative when the opposite face is upward.
2. **Gyro:** rotate the assembly roughly +90° about each positive BODY axis using the right-hand rule. The matching integrated gyro component must be positive and dominate the cross axes.

A failed sign is not a calibration issue; fix the sensor-to-body mapping.

### C. Magnetometer polarity

```sh
$PF_PY pre_flight/run.py polarity-magnetometer --port auto
```

Use a non-electronic compass to establish magnetic north. For each prompt, keep the requested body axis horizontal and point that positive or negative arrow north. The matching `b_body` component should change from positive to negative. This validates the complete raw-sensor → calibration → body-frame sign path without relying on the uncalibrated field magnitude.

Do this away from the magnetorquers, steel benches, speakers, laptops, and energized high-current wiring.

### D. Magnetometer calibration

```sh
$PF_PY pre_flight/run.py mag-calibrate --port auto --show
```

Rotate the **complete flight assembly** slowly through every orientation. Use figure-eights and make every body face point both up and down. The fit requires:

- at least 7 of 8 sampled octants;
- minimum/maximum axis span ratio of at least 0.45;
- corrected-radius coefficient of variation no more than 0.10.

The tool fits raw sensor-frame microtesla values, rejects strong radial outliers, plots raw and corrected point clouds, and prints `MAG_HARD_IRON_OFFSET` and `MAG_SOFT_IRON_MATRIX`. Copy both into both parameter files and rerun the capture for verification. Do not change `MAG_SENSOR_TO_BODY_SIGNS` as part of an ellipsoid fit.

### E. Sun-sensor polarity and channel mapping

```sh
$PF_PY pre_flight/run.py polarity-sun --port auto --show
```

First capture a covered/dim baseline. Then illuminate only the requested physical photodiode with a small flashlight. The addressed channel must have the largest positive increase. The expected channel labels are:

| Channel | Expected normal/label | Channel | Expected normal/label |
|---:|---|---:|---|
| 0 | P1-1, +X/+Z | 8 | -Y A |
| 1 | P1-2, +X/+Y | 9 | -Y B |
| 2 | P1-3, +X/-Z | 10 | +Y A |
| 3 | P1-4, +X/-Y | 11 | +Y B |
| 4 | P2-1, -X/+Z | 12 | +Z A |
| 5 | P2-2, -X/-Y | 13 | +Z B |
| 6 | P2-3, -X/-Z | 14 | -Z A |
| 7 | P2-4, -X/+Y | 15 | -Z B |

If physical labels differ from this table, stop and reconcile the schematic/harness before changing software. To test only selected channels while investigating:

```sh
$PF_PY pre_flight/run.py polarity-sun --channels 0,1,8,10 --port auto
```

### F. Magnetorquer polarity

```sh
$PF_PY pre_flight/run.py polarity-magnetorquer --port auto
```

This test provides two independent pieces of evidence:

- The onboard magnetometer measures the local field change for positive and negative commands. The changes must be nearly opposite, proving that both H-bridge directions work.
- The operator observes an external compass at the spacecraft's **positive axis end**. For a positive commanded dipole, the compass north-seeking end should point away from the spacecraft. This proves the absolute dipole sign; the onboard sensor alone cannot prove it because the local coil-to-sensor coupling geometry is not a body-axis reference.

If the compass is too slow for a 500 ms pulse, use an external digital magnetometer or current probe. Do not lengthen the firmware pulse limit merely to make observation easier.

## Final restore and evidence checklist

1. Save every JSON/PNG report with the flight configuration record.
2. Apply accepted calibration constants to both parameter files.
3. Rebuild and flash **`samwise-adcs-rewrite.uf2`**.
4. Power-cycle, confirm the expected artifact/version, and verify normal PiCubed telemetry.
5. Confirm the test image is no longer installed; `pre_flight/run.py check` should no longer receive a PONG from flight firmware.

## Protocol summary

The test image emits newline-delimited CSV prefixed by `PF`:

- `PF,MAG,time_ms,raw_x,raw_y,raw_z,body_x,body_y,body_z`
- `PF,IMU,time_ms,gx,gy,gz,ax,ay,az`
- `PF,SUN,time_ms,i0,...,i15`

Supported host commands are `PING`, `STATUS`, `CONFIG`, `STREAM MAG|IMU|SUN|ALL|OFF rate_hz`, `ARM MT`, `MT axis sign duty duration_ms`, `STOP`, and `DISARM`. The flight image does not contain this command parser.
