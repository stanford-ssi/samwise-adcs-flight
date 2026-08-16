#!/usr/bin/env python3
"""Guided SAMWISE ADCS calibration, visualization, and polarity checks."""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
from typing import Callable

import numpy as np

from lib.calibration import (
    apply_magnetometer_calibration,
    fit_accelerometer_six_face,
    fit_gyro,
    fit_magnetometer,
)
from lib.protocol import (
    Packet,
    PreflightLink,
    discover_ports,
    packet_sun,
    packet_vector,
)


ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
AXES = "XYZ"
SUN_LABELS = (
    "P1-1 +X/+Z", "P1-2 +X/+Y", "P1-3 +X/-Z", "P1-4 +X/-Y",
    "P2-1 -X/+Z", "P2-2 -X/-Y", "P2-3 -X/-Z", "P2-4 -X/+Y",
    "-Y A", "-Y B", "+Y A", "+Y B", "+Z A", "+Z B", "-Z A", "-Z B",
)


def timestamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def progress(current: int, total: int) -> None:
    print(f"\r  samples: {current}/{total}", end="", flush=True)
    if current == total:
        print()


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    raise TypeError(f"Cannot serialize {type(value)}")


def save_report(prefix: str, report: dict) -> Path:
    RESULTS.mkdir(parents=True, exist_ok=True)
    path = RESULTS / f"{prefix}_{timestamp()}.json"
    path.write_text(json.dumps(report, indent=2, default=json_value) + "\n")
    print(f"Report: {path}")
    return path


def connect(args) -> PreflightLink:
    link = PreflightLink(args.port, args.baud)
    try:
        version = link.verify()
        status = link.status()
    except Exception:
        link.close()
        raise
    print(f"Connected to {link.port} (protocol {version})")
    print("Sensors: " + ", ".join(f"{name}={'OK' if okay else 'FAIL'}" for name, okay in status.items() if name != "mt_armed"))
    return link


def require_sensor(status: dict[str, bool], *names: str) -> None:
    failed = [name for name in names if not status.get(name, False)]
    if failed:
        raise RuntimeError("Board reports unavailable hardware: " + ", ".join(failed))


def collect_vectors(
    link: PreflightLink,
    kind: str,
    section: str,
    count: int,
    rate_hz: int,
) -> np.ndarray:
    return np.asarray(
        link.collect(
            kind,
            count,
            rate_hz,
            lambda packet: packet_vector(packet, section),
            progress=progress,
        ),
        dtype=float,
    )


def collect_sun(link: PreflightLink, count: int = 30, rate_hz: int = 10) -> np.ndarray:
    return np.asarray(
        link.collect("SUN", count, rate_hz, packet_sun, progress=progress),
        dtype=float,
    )


def command_ports(_args) -> int:
    ports = discover_ports()
    if not ports:
        print("No serial ports found.")
        return 1
    for port, description in ports:
        print(f"{port}\t{description}")
    return 0


def command_check(args) -> int:
    with connect(args) as link:
        status = link.status()
        okay = all(status.get(name, False) for name in ("imu", "mag", "sun_pyramid", "sun_ads"))
        print("PASS" if okay else "FAIL")
        return 0 if okay else 2


def command_monitor(args) -> int:
    import matplotlib.pyplot as plt

    with connect(args) as link:
        link.send(f"STREAM ALL {args.rate}")
        link.wait_for(lambda packet: packet.kind == "OK" and packet.fields[:1] == ("STREAM",))

        history = args.seconds * args.rate
        times = deque(maxlen=history)
        mag = [deque(maxlen=history) for _ in range(3)]
        gyro = [deque(maxlen=history) for _ in range(3)]
        sun = np.zeros(16)
        start = time.monotonic()
        last_draw = 0.0

        plt.ion()
        figure, axes = plt.subplots(3, 1, figsize=(12, 9))
        figure.suptitle("SAMWISE ADCS pre-flight live monitor")
        print("Live monitor open; close the window or press Ctrl+C to stop.")
        try:
            while plt.fignum_exists(figure.number):
                packet = link.read_packet(time.monotonic() + 0.3)
                now = time.monotonic() - start
                if packet and packet.kind == "MAG":
                    values = packet_vector(packet, "body")
                    times.append(now)
                    for index, value in enumerate(values):
                        mag[index].append(value)
                elif packet and packet.kind == "IMU":
                    values = packet_vector(packet, "gyro")
                    for index, value in enumerate(values):
                        gyro[index].append(value)
                elif packet and packet.kind == "SUN":
                    sun = np.asarray(packet_sun(packet), dtype=float)

                if now - last_draw < 0.1:
                    continue
                last_draw = now
                for axis in axes:
                    axis.clear()
                mag_times = np.linspace(max(0, now - args.seconds), now, len(mag[0]))
                imu_times = np.linspace(max(0, now - args.seconds), now, len(gyro[0]))
                for index, label in enumerate(AXES):
                    axes[0].plot(mag_times, mag[index], label=label)
                    axes[1].plot(imu_times, gyro[index], label=label)
                axes[0].set_ylabel("B body (unit)")
                axes[1].set_ylabel("gyro (rad/s)")
                axes[0].legend(loc="upper right", ncol=3)
                axes[1].legend(loc="upper right", ncol=3)
                colors = ["tab:blue" if value < 500 else "tab:orange" for value in sun]
                axes[2].bar(np.arange(16), sun, color=colors)
                axes[2].set_xticks(np.arange(16), [str(i) for i in range(16)])
                axes[2].set_ylabel("sun intensity")
                axes[2].set_xlabel("sensor channel")
                figure.tight_layout()
                plt.pause(0.001)
        except KeyboardInterrupt:
            pass
        finally:
            link.send("STREAM OFF 1")
            plt.close(figure)
    return 0


def command_mag_calibrate(args) -> int:
    import matplotlib.pyplot as plt

    print("Keep torquers and motors OFF. Slowly rotate the complete flight assembly through all orientations.")
    print("Use smooth figure-eights and make sure every body face points up and down at least once.")
    input("Press Enter to begin capture...")
    with connect(args) as link:
        status = link.status()
        require_sensor(status, "mag")
        samples = collect_vectors(link, "MAG", "raw", args.samples, args.rate)

    calibration = fit_magnetometer(samples)
    corrected = apply_magnetometer_calibration(samples, calibration.center, calibration.transform)
    report = {
        "test": "magnetometer_calibration",
        "passed": calibration.passed,
        "samples": calibration.total_samples,
        "retained_samples": calibration.retained_samples,
        "hard_iron_offset_uT": calibration.center,
        "soft_iron_matrix": calibration.transform,
        "corrected_radius_mean": calibration.corrected_radius_mean,
        "corrected_radius_cv": calibration.corrected_radius_cv,
        "octants": calibration.octants,
        "span_ratio": calibration.span_ratio,
    }
    report_path = save_report("magnetometer_calibration", report)
    csv_path = report_path.with_suffix(".csv")
    np.savetxt(csv_path, samples, delimiter=",", header="raw_x_uT,raw_y_uT,raw_z_uT", comments="")

    figure = plt.figure(figsize=(12, 5))
    before = figure.add_subplot(121, projection="3d")
    after = figure.add_subplot(122, projection="3d")
    before.scatter(*samples.T, s=4, alpha=0.5)
    after.scatter(*corrected.T, s=4, alpha=0.5)
    before.set_title("Raw")
    after.set_title("Corrected")
    for axis in (before, after):
        axis.set_xlabel("X")
        axis.set_ylabel("Y")
        axis.set_zlabel("Z")
        axis.set_box_aspect((1, 1, 1))
    figure.suptitle("Magnetometer calibration: " + ("PASS" if calibration.passed else "RECAPTURE"))
    figure.tight_layout()
    plot_path = report_path.with_suffix(".png")
    figure.savefig(plot_path, dpi=160)
    if args.show:
        plt.show()
    else:
        plt.close(figure)

    center = calibration.center
    matrix = calibration.transform
    print("\nCopy into BOTH parameter files (the shared driver currently reads adcs_app):")
    print(f"constexpr float3 MAG_HARD_IRON_OFFSET = float3{{{center[0]:.7f}f, {center[1]:.7f}f, {center[2]:.7f}f}};")
    print("constexpr float3x3 MAG_SOFT_IRON_MATRIX = {")
    for row in matrix:
        print("    {" + ", ".join(f"{value:.9f}f" for value in row) + "},")
    print("};")
    print(f"Quality: octants={calibration.octants}/8, span ratio={calibration.span_ratio:.3f}, radius CV={calibration.corrected_radius_cv:.3f}")
    print("PASS" if calibration.passed else "RECAPTURE: orientation coverage or fit quality is insufficient")
    print(f"Plot: {plot_path}")
    return 0 if calibration.passed else 2


def command_imu_calibrate(args) -> int:
    print("Place the complete assembly on a rigid surface and do not touch it during capture.")
    input("Press Enter once it is stationary...")
    with connect(args) as link:
        status = link.status()
        require_sensor(status, "imu")
        existing = np.asarray(link.imu_zero())
        gyro_samples = collect_vectors(link, "IMU", "gyro", args.samples, args.rate)
        gyro = fit_gyro(gyro_samples, existing)

        face_means: dict[str, np.ndarray] = {}
        if args.six_face:
            for axis_index, axis in enumerate("xyz"):
                for sign in ("+", "-"):
                    orientation = f"{sign}{axis}"
                    print(f"\nPlace the {orientation.upper()} BODY face upward and hold still.")
                    input("Press Enter to capture...")
                    values = collect_vectors(link, "IMU", "accel", args.face_samples, args.rate)
                    face_means[orientation] = np.mean(values, axis=0)

    report = {
        "test": "imu_calibration",
        "passed": not gyro.moving,
        "existing_gyro_offset_rad_s": existing,
        "observed_residual_mean_rad_s": gyro.observed_mean,
        "gyro_std_rad_s": gyro.observed_std,
        "new_gyro_offset_rad_s": gyro.new_offset,
    }
    if face_means:
        accel = fit_accelerometer_six_face(face_means)
        report.update(
            {
                "accelerometer_passed": accel.passed,
                "accelerometer_face_means_km_s2": face_means,
                "accelerometer_bias_km_s2": accel.bias,
                "accelerometer_scale_measured_per_true": accel.scale,
                "accelerometer_cross_axis_ratio": accel.cross_axis_ratio,
            }
        )
        report["passed"] = bool(report["passed"] and accel.passed)

    save_report("imu_calibration", report)
    offset = gyro.new_offset
    print("\nCopy into BOTH parameter files:")
    print(f"constexpr float3 IMU_ZERO_READING_RPS = {{{offset[0]:.9f}f, {offset[1]:.9f}f, {offset[2]:.9f}f}};")
    print("Gyro noise std [rad/s]: " + np.array2string(gyro.observed_std, precision=8))
    if gyro.moving:
        print("FAIL: motion detected; repeat on a stable surface")
    elif face_means:
        print("Accelerometer characterization is reported but is NOT yet consumed by the flight driver.")
        print("PASS" if report["passed"] else "FAIL: inspect accelerometer scale/cross-axis result")
    else:
        print("PASS")
    return 0 if report["passed"] else 2


def command_polarity_magnetometer(args) -> int:
    import matplotlib.pyplot as plt

    print("This checks absolute BODY-axis signs using local magnetic north.")
    print("Use a compass, remove magnets/steel, and keep each requested body axis horizontal.")
    captures: dict[str, np.ndarray] = {}
    with connect(args) as link:
        require_sensor(link.status(), "mag")
        for axis in AXES:
            for sign, word in (("+", "positive"), ("-", "negative")):
                print(f"\nPoint the {word} BODY {axis} axis toward magnetic north.")
                input("Hold still and press Enter...")
                values = collect_vectors(link, "MAG", "body", args.samples, args.rate)
                captures[f"{sign}{axis}"] = np.mean(values, axis=0)

    results = {}
    for index, axis in enumerate(AXES):
        plus = captures[f"+{axis}"][index]
        minus = captures[f"-{axis}"][index]
        results[axis] = {
            "plus_axis_north_component": plus,
            "minus_axis_north_component": minus,
            "separation": plus - minus,
            "passed": bool(plus > 0 and minus < 0 and plus - minus > 0.5),
        }
    passed = all(item["passed"] for item in results.values())
    report_path = save_report("magnetometer_polarity", {"test": "magnetometer_polarity", "passed": passed, "captures": captures, "axes": results})

    x = np.arange(3)
    figure, axis = plt.subplots(figsize=(8, 5))
    axis.bar(x - 0.18, [captures[f"+{name}"][i] for i, name in enumerate(AXES)], 0.36, label="+axis north")
    axis.bar(x + 0.18, [captures[f"-{name}"][i] for i, name in enumerate(AXES)], 0.36, label="-axis north")
    axis.axhline(0, color="black", linewidth=0.8)
    axis.set_xticks(x, AXES)
    axis.set_ylabel("matching B-body component")
    axis.legend()
    axis.set_title("Magnetometer polarity: " + ("PASS" if passed else "FAIL"))
    figure.tight_layout()
    figure.savefig(report_path.with_suffix(".png"), dpi=160)
    if args.show:
        plt.show()
    else:
        plt.close(figure)
    for axis_name, result in results.items():
        print(f"{axis_name}: {'PASS' if result['passed'] else 'FAIL'} ({result['plus_axis_north_component']:+.3f}, {result['minus_axis_north_component']:+.3f})")
    return 0 if passed else 2


def command_polarity_imu(args) -> int:
    print("Accelerometer: each BODY face is tested against gravity/support force.")
    accel_means: dict[str, np.ndarray] = {}
    gyro_runs: dict[str, np.ndarray] = {}
    with connect(args) as link:
        require_sensor(link.status(), "imu")
        for axis in "xyz":
            for sign in ("+", "-"):
                key = f"{sign}{axis}"
                print(f"\nPlace the {key.upper()} BODY face upward and hold still.")
                input("Press Enter to capture...")
                samples = collect_vectors(link, "IMU", "accel", args.face_samples, args.rate)
                accel_means[key] = np.mean(samples, axis=0)

        print("\nGyro sign test: use the right-hand rule around each positive BODY axis.")
        for axis in AXES:
            print(f"\nPrepare to rotate roughly +90 degrees about +{axis} during the capture window.")
            input("Press Enter, then rotate smoothly now...")
            gyro_runs[axis] = collect_vectors(link, "IMU", "gyro", args.gyro_samples, args.rate)

    accel_fit = fit_accelerometer_six_face(accel_means)
    gyro_results = {}
    for index, axis in enumerate(AXES):
        run = gyro_runs[axis]
        signed_area = np.sum(run, axis=0) / args.rate
        cross = np.max(np.abs(np.delete(signed_area, index)))
        gyro_results[axis] = {
            "integrated_rad": signed_area,
            "passed": bool(signed_area[index] > 0.3 and signed_area[index] > 1.5 * cross),
        }
    passed = accel_fit.passed and all(item["passed"] for item in gyro_results.values())
    save_report(
        "imu_polarity",
        {
            "test": "imu_polarity",
            "passed": passed,
            "accelerometer_face_means_km_s2": accel_means,
            "accelerometer_scale": accel_fit.scale,
            "accelerometer_bias_km_s2": accel_fit.bias,
            "accelerometer_cross_axis_ratio": accel_fit.cross_axis_ratio,
            "accelerometer_passed": accel_fit.passed,
            "gyro": gyro_results,
        },
    )
    print(f"Accelerometer: {'PASS' if accel_fit.passed else 'FAIL'}; scale={accel_fit.scale}, cross-axis={accel_fit.cross_axis_ratio:.3f}")
    for axis, result in gyro_results.items():
        print(f"Gyro {axis}: {'PASS' if result['passed'] else 'FAIL'}; integrated={result['integrated_rad']}")
    return 0 if passed else 2


def command_polarity_sun(args) -> int:
    import matplotlib.pyplot as plt

    channels = list(range(16)) if args.channels == "all" else [int(item) for item in args.channels.split(",")]
    if any(channel < 0 or channel >= 16 for channel in channels):
        raise ValueError("Sun sensor channels must be between 0 and 15")
    print("Cover the assembly or place it in uniform dim light for the baseline.")
    input("Press Enter to capture baseline...")
    with connect(args) as link:
        require_sensor(link.status(), "sun_pyramid", "sun_ads")
        baseline = np.median(collect_sun(link, args.samples, args.rate), axis=0)
        deltas = {}
        for channel in channels:
            print(f"\nIlluminate ONLY channel {channel}: {SUN_LABELS[channel]}")
            input("Press Enter while the light is steady...")
            lit = np.median(collect_sun(link, args.samples, args.rate), axis=0)
            deltas[channel] = lit - baseline

    results = {}
    for channel, delta in deltas.items():
        winner = int(np.argmax(delta))
        results[channel] = {
            "label": SUN_LABELS[channel],
            "winning_channel": winner,
            "target_delta": float(delta[channel]),
            "passed": bool(winner == channel and delta[channel] >= args.min_delta),
        }
    passed = all(item["passed"] for item in results.values())
    report_path = save_report("sun_sensor_polarity", {"test": "sun_sensor_polarity", "passed": passed, "baseline": baseline, "deltas": deltas, "channels": results})

    matrix = np.vstack([deltas[channel] for channel in channels])
    figure, axis = plt.subplots(figsize=(12, max(4, len(channels) * 0.45)))
    image = axis.imshow(matrix, aspect="auto", cmap="viridis")
    axis.set_xticks(np.arange(16), [str(index) for index in range(16)])
    axis.set_yticks(np.arange(len(channels)), [str(channel) for channel in channels])
    axis.set_xlabel("responding channel")
    axis.set_ylabel("illuminated channel")
    axis.set_title("Sun sensor channel response")
    figure.colorbar(image, ax=axis, label="intensity above baseline")
    figure.tight_layout()
    figure.savefig(report_path.with_suffix(".png"), dpi=160)
    if args.show:
        plt.show()
    else:
        plt.close(figure)
    for channel, result in results.items():
        print(f"{channel:2d} {result['label']}: {'PASS' if result['passed'] else 'FAIL'} (winner {result['winning_channel']}, delta {result['target_delta']:.0f})")
    return 0 if passed else 2


def command_polarity_magnetorquer(args) -> int:
    if not 0.01 <= args.duty <= 0.25:
        raise ValueError("Magnetorquer duty must be between 0.01 and 0.25")
    if not 150 <= args.duration <= 500:
        raise ValueError("Magnetorquer duration must be between 150 and 500 ms")
    print("DANGER: this energizes one rod at a time.")
    print("Confirm the magnetorquer rail may be powered during USB testing, keep ferrous objects clear,")
    print("and place a compass at the spacecraft +axis end. For a positive dipole, its north end should point away.")
    if input("Type ARM to continue: ").strip() != "ARM":
        print("Aborted without arming.")
        return 1

    results = {}
    manual = {}
    with connect(args) as link:
        require_sensor(link.status(), "mag")
        link.send("STREAM OFF 1")
        for axis in AXES:
            axis_deltas = {}
            for sign in (1, -1):
                print(f"\nReady for {axis} {'positive' if sign > 0 else 'negative'} pulse.")
                input("Press Enter to pulse...")
                link.send("ARM MT")
                link.wait_for(lambda packet: packet.kind == "OK" and packet.fields[:1] == ("MT_ARMED",))
                link.send(f"MT {axis} {sign} {args.duty:.3f} {args.duration}")
                packet = link.wait_for(
                    lambda item: item.kind == "MT" and item.fields[:1] == ("RESULT",),
                    timeout=5.0,
                )
                delta = np.asarray([float(value) for value in packet.fields[5:8]])
                if delta.shape != (3,):
                    raise RuntimeError(f"Malformed magnetorquer result: {packet.raw_line}")
                axis_deltas[sign] = delta
                print(f"Onboard field delta [uT]: {delta}")
                if sign > 0:
                    answer = input("Did the external compass indicate a +BODY-axis dipole? [y/n/u]: ").strip().lower()
                    manual[axis] = answer

            plus = axis_deltas[1]
            minus = axis_deltas[-1]
            plus_norm = float(np.linalg.norm(plus))
            minus_norm = float(np.linalg.norm(minus))
            cosine = float(np.dot(plus, minus) / (plus_norm * minus_norm)) if plus_norm and minus_norm else 1.0
            ratio = plus_norm / minus_norm if minus_norm else float("inf")
            relative_pass = bool(cosine < -0.8 and 0.5 <= ratio <= 2.0)
            absolute_pass = manual[axis] == "y"
            results[axis] = {
                "positive_delta_uT": plus,
                "negative_delta_uT": minus,
                "opposition_cosine": cosine,
                "magnitude_ratio": ratio,
                "relative_reversal_passed": relative_pass,
                "absolute_manual_result": manual[axis],
                "passed": relative_pass and absolute_pass,
            }
        link.send("DISARM")

    passed = all(item["passed"] for item in results.values())
    save_report("magnetorquer_polarity", {"test": "magnetorquer_polarity", "passed": passed, "duty": args.duty, "duration_ms": args.duration, "axes": results})
    for axis, result in results.items():
        print(f"{axis}: {'PASS' if result['passed'] else 'FAIL/UNVERIFIED'}; reversal cosine={result['opposition_cosine']:.3f}, ratio={result['magnitude_ratio']:.2f}, absolute={result['absolute_manual_result']}")
    return 0 if passed else 2


def add_connection_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", default="auto", help="serial port or 'auto' (default)")
    parser.add_argument("--baud", type=int, default=115200, help="USB CDC baud placeholder")


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    ports = subparsers.add_parser("ports", help="list serial ports")
    ports.set_defaults(func=command_ports)

    check = subparsers.add_parser("check", help="verify firmware and sensor initialization")
    add_connection_arguments(check)
    check.set_defaults(func=command_check)

    monitor = subparsers.add_parser("monitor", help="live magnetometer/IMU/sun-sensor dashboard")
    add_connection_arguments(monitor)
    monitor.add_argument("--rate", type=int, default=10)
    monitor.add_argument("--seconds", type=int, default=20, help="plot history")
    monitor.set_defaults(func=command_monitor)

    mag_cal = subparsers.add_parser("mag-calibrate", help="fit hard/soft-iron calibration")
    add_connection_arguments(mag_cal)
    mag_cal.add_argument("--samples", type=int, default=1500)
    mag_cal.add_argument("--rate", type=int, default=25)
    mag_cal.add_argument("--show", action="store_true")
    mag_cal.set_defaults(func=command_mag_calibrate)

    imu_cal = subparsers.add_parser("imu-calibrate", help="calibrate gyro bias and optionally characterize accelerometer")
    add_connection_arguments(imu_cal)
    imu_cal.add_argument("--samples", type=int, default=500)
    imu_cal.add_argument("--rate", type=int, default=50)
    imu_cal.add_argument("--six-face", action="store_true", help="also run accelerometer six-face characterization")
    imu_cal.add_argument("--face-samples", type=int, default=150)
    imu_cal.set_defaults(func=command_imu_calibrate)

    mag_pol = subparsers.add_parser("polarity-magnetometer", help="guided absolute magnetometer BODY-axis test")
    add_connection_arguments(mag_pol)
    mag_pol.add_argument("--samples", type=int, default=50)
    mag_pol.add_argument("--rate", type=int, default=25)
    mag_pol.add_argument("--show", action="store_true")
    mag_pol.set_defaults(func=command_polarity_magnetometer)

    imu_pol = subparsers.add_parser("polarity-imu", help="guided accelerometer and gyro BODY-axis test")
    add_connection_arguments(imu_pol)
    imu_pol.add_argument("--rate", type=int, default=50)
    imu_pol.add_argument("--face-samples", type=int, default=100)
    imu_pol.add_argument("--gyro-samples", type=int, default=150)
    imu_pol.set_defaults(func=command_polarity_imu)

    sun_pol = subparsers.add_parser("polarity-sun", help="guided sun-sensor channel mapping test")
    add_connection_arguments(sun_pol)
    sun_pol.add_argument("--channels", default="all", help="'all' or comma-separated channel numbers")
    sun_pol.add_argument("--samples", type=int, default=20)
    sun_pol.add_argument("--rate", type=int, default=10)
    sun_pol.add_argument("--min-delta", type=float, default=300)
    sun_pol.add_argument("--show", action="store_true")
    sun_pol.set_defaults(func=command_polarity_sun)

    mt_pol = subparsers.add_parser("polarity-magnetorquer", help="guided bounded rod-polarity test")
    add_connection_arguments(mt_pol)
    mt_pol.add_argument("--duty", type=float, default=0.15, help="fractional duty, maximum 0.25")
    mt_pol.add_argument("--duration", type=int, default=500, help="pulse duration in ms, 150-500")
    mt_pol.set_defaults(func=command_polarity_magnetorquer)

    return parser


def main() -> int:
    parser = make_parser()
    args = parser.parse_args()
    try:
        return int(args.func(args))
    except KeyboardInterrupt:
        print("\nInterrupted; STOP/DISARM sent while closing the USB link.", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
