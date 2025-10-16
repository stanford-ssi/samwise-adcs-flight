#!/usr/bin/env python3
"""
IMU Calibration Script for SAMWISE ADCS

This script reads IMU data from serial output and calculates the zero-rate bias offset.
The IMU should be stationary during calibration.

Usage:
    python imu_calibration.py [--port PORT] [--baud BAUD] [--samples N]

    Or pipe serial data:
    cat serial_log.txt | python imu_calibration.py --file -

Example serial line format:
    [INFO]    [sensors] IMU reading: [0.0019308, -0.0015979, -0.0005326] | Filtered: [0.0020259, -0.0016460, -0.0004759] | Magnitude: 0.0026533
"""

import re
import sys
import argparse
import numpy as np
from collections import deque

def parse_imu_line(line):
    """
    Parse IMU reading from a log line.

    Returns:
        tuple: (x, y, z) raw IMU readings in rad/s, or None if not found
    """
    # Match pattern: [sensors] IMU reading: [x, y, z] | ...
    pattern = r'\[sensors\]\s+IMU reading:\s+\[([-\d.]+),\s+([-\d.]+),\s+([-\d.]+)\]'
    match = re.search(pattern, line)

    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        z = float(match.group(3))
        return (x, y, z)

    return None

def calculate_statistics(samples):
    """
    Calculate mean, standard deviation, and confidence intervals.

    Args:
        samples: List of (x, y, z) tuples

    Returns:
        dict: Statistics including mean, std, min, max
    """
    samples_array = np.array(samples)

    stats = {
        'mean': np.mean(samples_array, axis=0),
        'std': np.std(samples_array, axis=0),
        'min': np.min(samples_array, axis=0),
        'max': np.max(samples_array, axis=0),
        'count': len(samples)
    }

    return stats

def read_from_serial(port, baud, num_samples):
    """
    Read IMU data directly from serial port.

    Args:
        port: Serial port (e.g., '/dev/ttyACM0')
        baud: Baud rate
        num_samples: Number of samples to collect

    Returns:
        list: Collected samples
    """
    try:
        import serial
    except ImportError:
        print("Error: pyserial not installed. Install with: pip install pyserial")
        sys.exit(1)

    samples = []

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to {port} at {baud} baud")
        print(f"Collecting {num_samples} samples...")
        print("Ensure the IMU is stationary during calibration!\n")

        while len(samples) < num_samples:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if line:
                    reading = parse_imu_line(line)
                    if reading:
                        samples.append(reading)
                        if len(samples) % 10 == 0:
                            print(f"Progress: {len(samples)}/{num_samples} samples collected")
                            print(f"  Latest: [{reading[0]:.7f}, {reading[1]:.7f}, {reading[2]:.7f}]")

            except UnicodeDecodeError:
                continue

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)
    finally:
        if 'ser' in locals():
            ser.close()

    return samples

def read_from_file(file_handle, num_samples):
    """
    Read IMU data from a file or stdin.

    Args:
        file_handle: File object to read from
        num_samples: Number of samples to collect (None for all)

    Returns:
        list: Collected samples
    """
    samples = []

    print("Reading IMU data from file/stdin...")
    print("Ensure the IMU was stationary when data was collected!\n")

    for line in file_handle:
        line = line.strip()
        if line:
            reading = parse_imu_line(line)
            if reading:
                samples.append(reading)

                if num_samples and len(samples) >= num_samples:
                    break

                if len(samples) % 50 == 0:
                    print(f"Progress: {len(samples)} samples collected")

    return samples

def detect_motion(samples, threshold=0.01):
    """
    Detect if there was significant motion during calibration.

    Args:
        samples: List of (x, y, z) tuples
        threshold: Motion threshold in rad/s

    Returns:
        bool: True if motion detected
    """
    samples_array = np.array(samples)
    std = np.std(samples_array, axis=0)

    # If any axis has high standard deviation, motion likely occurred
    return np.any(std > threshold)

def main():
    parser = argparse.ArgumentParser(
        description='IMU Calibration Script for SAMWISE ADCS',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Read from serial port
  python imu_calibration.py --port /dev/ttyACM0 --samples 200

  # Read from log file
  python imu_calibration.py --file serial_log.txt --samples 200

  # Pipe from serial
  cat /dev/ttyACM0 | python imu_calibration.py --file - --samples 200
        """
    )

    parser.add_argument('--port', type=str,
                        help='Serial port (e.g., /dev/ttyACM0 or COM3)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--file', type=str,
                        help='Read from file instead of serial port (use "-" for stdin)')
    parser.add_argument('--samples', type=int, default=200,
                        help='Number of samples to collect (default: 200)')
    parser.add_argument('--motion-threshold', type=float, default=0.01,
                        help='Motion detection threshold in rad/s (default: 0.01)')

    args = parser.parse_args()

    # Validate arguments
    if not args.port and not args.file:
        parser.error("Must specify either --port or --file")

    if args.port and args.file:
        parser.error("Cannot specify both --port and --file")

    # Collect samples
    samples = []

    if args.port:
        samples = read_from_serial(args.port, args.baud, args.samples)
    elif args.file:
        if args.file == '-':
            samples = read_from_file(sys.stdin, args.samples)
        else:
            try:
                with open(args.file, 'r') as f:
                    samples = read_from_file(f, args.samples)
            except FileNotFoundError:
                print(f"Error: File '{args.file}' not found")
                sys.exit(1)

    if len(samples) == 0:
        print("\nError: No IMU readings found!")
        print("Make sure the serial output contains lines like:")
        print("  [INFO]    [sensors] IMU reading: [x, y, z] | ...")
        sys.exit(1)

    print(f"\n{'='*70}")
    print(f"Calibration complete! Collected {len(samples)} samples")
    print(f"{'='*70}\n")

    # Calculate statistics
    stats = calculate_statistics(samples)

    # Check for motion
    motion_detected = detect_motion(samples, args.motion_threshold)
    if motion_detected:
        print("⚠️  WARNING: Significant motion detected during calibration!")
        print("   The IMU may not have been stationary. Consider recalibrating.\n")

    # Print results
    print("Calibration Results:")
    print(f"  Samples collected: {stats['count']}")
    print(f"\n  Bias (mean) [rad/s]:")
    print(f"    X: {stats['mean'][0]:+.7f}")
    print(f"    Y: {stats['mean'][1]:+.7f}")
    print(f"    Z: {stats['mean'][2]:+.7f}")

    print(f"\n  Standard deviation [rad/s]:")
    print(f"    X: {stats['std'][0]:.7f}")
    print(f"    Y: {stats['std'][1]:.7f}")
    print(f"    Z: {stats['std'][2]:.7f}")

    print(f"\n  Range [rad/s]:")
    print(f"    X: [{stats['min'][0]:+.7f}, {stats['max'][0]:+.7f}]")
    print(f"    Y: [{stats['min'][1]:+.7f}, {stats['max'][1]:+.7f}]")
    print(f"    Z: [{stats['min'][2]:+.7f}, {stats['max'][2]:+.7f}]")

    # Generate C++ constant
    print(f"\n{'='*70}")
    print("Update the following line in src/constants.h:")
    print(f"{'='*70}")
    print(f"\nconstexpr float3 IMU_ZERO_READING_RPS = {{{stats['mean'][0]:.7f}f, {stats['mean'][1]:.7f}f, {stats['mean'][2]:.7f}f}};")
    print()

    # Additional analysis
    magnitude = np.linalg.norm(stats['mean'])
    print(f"Bias magnitude: {magnitude:.7f} rad/s ({magnitude * 180 / np.pi:.4f} deg/s)")

    # Estimate noise characteristics
    noise_std = np.mean(stats['std'])
    print(f"\nNoise characteristics:")
    print(f"  Average noise (1σ): {noise_std:.7f} rad/s ({noise_std * 180 / np.pi:.4f} deg/s)")
    print(f"  Estimated noise (3σ): {3*noise_std:.7f} rad/s ({3*noise_std * 180 / np.pi:.4f} deg/s)")

    return 0

if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user")
        sys.exit(1)
