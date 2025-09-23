#!/usr/bin/env python3
"""
Real-time satellite orientation plotter using quaternion data from EKF.
Plots 3 orthogonal vectors representing the satellite body frame in ECI coordinates.

Usage: python plot_orientation.py [--port /dev/cu.usbmodem101]

Expected input format: [INFO] qx, qy, qz, qw, extra_value
where quaternion is in scalar-last format (qx, qy, qz, qw)
"""

import serial
import matplotlib.pyplot as plt
import numpy as np
import argparse
from mpl_toolkits.mplot3d import Axes3D

# Configuration constants
UPDATE_RATE_HZ = 20  # Update rate in Hz
FRAME_PERIOD = 1.0 / UPDATE_RATE_HZ  # Period between frame updates

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """
    Convert quaternion (scalar-last format) to rotation matrix.
    Returns 3x3 rotation matrix that transforms from body frame to ECI frame.
    """
    # Normalize quaternion
    norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    # Compute rotation matrix elements
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
    ])

    return R

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Plot satellite orientation from quaternion data')
    parser.add_argument("--port", type=str, required=False, default="/dev/cu.usbmodem101",
                       help="Serial port to read from")
    args = parser.parse_args()

    # Setup serial connection
    try:
        ser = serial.Serial(args.port, 115200, timeout=1)
        print(f"Connected to {args.port}")
    except Exception as e:
        print(f"Failed to connect to {args.port}: {e}")
        return

    # Setup 3D plot
    plt.ion()
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Body frame vectors (unit vectors along x, y, z axes)
    body_x = np.array([1, 0, 0])
    body_y = np.array([0, 1, 0])
    body_z = np.array([0, 0, 1])

    # Initialize plot elements
    origin = np.array([0, 0, 0])

    # Plot body frame vectors
    x_line = ax.quiver(origin[0], origin[1], origin[2],
                      body_x[0], body_x[1], body_x[2],
                      color='red', arrow_length_ratio=0.1, linewidth=3, label='Body X')
    y_line = ax.quiver(origin[0], origin[1], origin[2],
                      body_y[0], body_y[1], body_y[2],
                      color='green', arrow_length_ratio=0.1, linewidth=3, label='Body Y')
    z_line = ax.quiver(origin[0], origin[1], origin[2],
                      body_z[0], body_z[1], body_z[2],
                      color='blue', arrow_length_ratio=0.1, linewidth=3, label='Body Z')

    # Setup plot appearance
    ax.set_xlabel('ECI X')
    ax.set_ylabel('ECI Y')
    ax.set_zlabel('ECI Z')
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.legend()
    ax.set_title('Satellite Body Frame Orientation in ECI')

    # Add reference ECI axes (thinner, gray)
    ax.quiver(0, 0, 0, 1, 0, 0, color='gray', alpha=0.3, linewidth=1)
    ax.quiver(0, 0, 0, 0, 1, 0, color='gray', alpha=0.3, linewidth=1)
    ax.quiver(0, 0, 0, 0, 0, 1, color='gray', alpha=0.3, linewidth=1)

    import time
    last_update_time = time.time()

    try:
        print("Waiting for quaternion data...")
        print("Expected format: [INFO] qx, qy, qz, qw, extra_value")

        while True:
            current_time = time.time()

            # Read serial data
            if ser.in_waiting:
                try:
                    # Read and parse line
                    line_raw = ser.readline().decode('utf-8', errors='replace').strip()

                    # Check if line contains [INFO]
                    if "[INFO]" not in line_raw:
                        continue

                    line_after_info = line_raw.split("[INFO]")[-1].strip()

                    # Extract numbers
                    number_strings = [s.strip() for s in line_after_info.split(",") if s.strip()]
                    nums = [float(n) for n in number_strings]

                    # Verify we have at least 4 values for quaternion
                    if len(nums) < 4:
                        print(f"Warning: Need at least 4 values for quaternion, got {len(nums)}")
                        continue

                    # Extract quaternion (first 4 values, scalar-last format)
                    qx, qy, qz, qw = nums[0], nums[1], nums[2], nums[3]

                    # Rate limiting: only update at specified rate
                    if current_time - last_update_time >= FRAME_PERIOD:
                        # Convert quaternion to rotation matrix
                        R = quaternion_to_rotation_matrix(qx, qy, qz, qw)

                        # Transform body frame vectors to ECI frame
                        eci_x = R @ body_x
                        eci_y = R @ body_y
                        eci_z = R @ body_z

                        # Clear and redraw arrows
                        ax.clear()

                        # Plot transformed body frame vectors
                        ax.quiver(0, 0, 0, eci_x[0], eci_x[1], eci_x[2],
                                 color='red', arrow_length_ratio=0.1, linewidth=3, label='Body X')
                        ax.quiver(0, 0, 0, eci_y[0], eci_y[1], eci_y[2],
                                 color='green', arrow_length_ratio=0.1, linewidth=3, label='Body Y')
                        ax.quiver(0, 0, 0, eci_z[0], eci_z[1], eci_z[2],
                                 color='blue', arrow_length_ratio=0.1, linewidth=3, label='Body Z')

                        # Add reference ECI axes
                        ax.quiver(0, 0, 0, 1, 0, 0, color='gray', alpha=0.3, linewidth=1, label='ECI X')
                        ax.quiver(0, 0, 0, 0, 1, 0, color='gray', alpha=0.3, linewidth=1, label='ECI Y')
                        ax.quiver(0, 0, 0, 0, 0, 1, color='gray', alpha=0.3, linewidth=1, label='ECI Z')

                        # Restore plot settings
                        ax.set_xlabel('ECI X')
                        ax.set_ylabel('ECI Y')
                        ax.set_zlabel('ECI Z')
                        ax.set_xlim([-1.5, 1.5])
                        ax.set_ylim([-1.5, 1.5])
                        ax.set_zlim([-1.5, 1.5])
                        ax.legend()
                        ax.set_title(f'Satellite Orientation (q=[{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}])')

                        plt.draw()
                        plt.pause(0.01)
                        last_update_time = current_time

                        print(f"Updated: q=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})")

                except Exception as e:
                    print(f"Error parsing line '{line_raw}': {e}")

            else:
                # No data available, small delay to prevent busy waiting
                time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()