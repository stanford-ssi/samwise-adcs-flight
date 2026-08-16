# ADCS pre-flight operator checklist

Vehicle: ____________________  Operator: ____________________  UTC: ____________________

Test image SHA/artifact: ____________________  Flight image SHA/artifact: ____________________

## Setup

- [ ] Vehicle mechanically supported; loose ferrous objects removed.
- [ ] Electrical owner confirmed the approved power source for rod testing.
- [ ] `samwise-adcs-preflight.uf2` flashed.
- [ ] `pre_flight/run.py check --port _____` reports IMU, MAG, SUN_PYRAMID, and SUN_ADS OK.
- [ ] Live monitor shows changing, finite data on all sensor groups.

## Calibration

- [ ] IMU gyro stationary calibration PASS; JSON attached.
- [ ] `IMU_ZERO_READING_RPS` copied into both parameter files.
- [ ] IMU calibration rebuilt and residual capture PASS.
- [ ] Optional accelerometer six-face characterization reviewed.
- [ ] Magnetometer capture covers at least 7/8 octants.
- [ ] Magnetometer span ratio ≥ 0.45.
- [ ] Magnetometer corrected-radius CV ≤ 0.10.
- [ ] Magnetometer constants copied into both parameter files.
- [ ] Magnetometer calibration rebuilt and verification capture PASS.

## Polarity and mapping

- [ ] IMU accelerometer +X/-X, +Y/-Y, +Z/-Z checks PASS.
- [ ] IMU right-hand-rule +X, +Y, +Z gyro checks PASS.
- [ ] Magnetometer +X/-X, +Y/-Y, +Z/-Z magnetic-north checks PASS.
- [ ] All 16 sun channels increase on the expected physical sensor only.
- [ ] Magnetorquer +/− onboard field deltas are opposite on X, Y, and Z.
- [ ] External compass/digital magnetometer confirms absolute +X dipole.
- [ ] External compass/digital magnetometer confirms absolute +Y dipole.
- [ ] External compass/digital magnetometer confirms absolute +Z dipole.
- [ ] Magnetorquers DISARMED and test supply returned to safe state.

## Restore flight configuration

- [ ] Calibration/polarity reports archived with vehicle configuration.
- [ ] `samwise-adcs-flight.uf2` rebuilt from the recorded commit.
- [ ] `samwise-adcs-flight.uf2` flashed; pre-flight image removed.
- [ ] Vehicle power-cycled.
- [ ] PiCubed receives and decodes the expected 77-byte telemetry packet.
- [ ] Final artifact hash recorded: ________________________________________________

Disposition:  [ ] GO  [ ] NO-GO

Operator signature: ____________________  Reviewer signature: ____________________
