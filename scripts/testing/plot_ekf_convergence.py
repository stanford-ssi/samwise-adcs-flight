#!/usr/bin/env python3
"""
Read EKF convergence data from serial, save to CSV, and plot.

Usage:
    python3 scripts/plot_ekf_convergence.py [serial_port]

Examples:
    python3 scripts/plot_ekf_convergence.py /dev/tty.usbmodem14201
    python3 scripts/plot_ekf_convergence.py COM3
    python3 scripts/plot_ekf_convergence.py  # Auto-detect port

The script will:
- Connect to serial port and read data
- Save raw log to data/ekf_raw_log.txt
- Save clean CSV to data/ekf_convergence.csv
- Generate plots to plots/ekf_convergence.png
"""

import sys
import matplotlib.pyplot as plt
import numpy as np
import csv
import serial
import serial.tools.list_ports
import time
from pathlib import Path


def find_serial_port(port=None):
    """Find the serial port to use."""
    if port:
        return port

    # Auto-detect Pico
    ports = serial.tools.list_ports.comports()
    for p in ports:
        # Look for Pico or Raspberry Pi in description
        if 'usbmodem' in p.device.lower() or 'pico' in p.description.lower():
            print(f"Auto-detected port: {p.device} ({p.description})")
            return p.device

    # Fall back to first available port
    if ports:
        print(f"Using first available port: {ports[0].device}")
        return ports[0].device

    raise ValueError("No serial ports found!")


def read_from_serial(port, baudrate=115200):
    """
    Read data from serial port until DATA_END marker.

    Args:
        port: Serial port path
        baudrate: Baud rate (default 115200)

    Returns:
        List of lines read from serial
    """
    print(f"Connecting to {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(2)  # Wait for connection to stabilize

    print("Waiting for DATA_START marker...")
    lines = []
    in_data = False
    start_time = time.time()

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"[SERIAL] {line}")  # Echo to console
                lines.append(line)

                if 'DATA_START' in line:
                    in_data = True
                    print("\n*** DATA COLLECTION STARTED ***\n")

                if 'DATA_END' in line:
                    print("\n*** DATA COLLECTION COMPLETE ***\n")
                    break

        except KeyboardInterrupt:
            print("\nInterrupted by user")
            break
        except Exception as e:
            print(f"Error reading serial: {e}")
            continue

    ser.close()
    return lines


def parse_ekf_log(lines):
    """
    Parse EKF convergence log data from lines.

    Args:
        lines: List of log lines

    Returns:
        Dictionary with numpy arrays for each column
    """
    # Find DATA_START and DATA_END markers
    data_lines = []
    in_data = False

    for line in lines:
        if 'DATA_START' in line:
            in_data = True
            continue
        if 'DATA_END' in line:
            break
        if in_data and line.strip() and not line.startswith('['):
            # Skip header and logging lines
            if not line.startswith('step,') and ',' in line:
                data_lines.append(line.strip())

    if not data_lines:
        raise ValueError("No data found between DATA_START and DATA_END markers")

    # Parse CSV data
    # Format: step,time,P_log_frobenius,mrp_error,quat_error,p_x,p_y,p_z,q_w,q_x,q_y,q_z,b_x,b_y,b_z
    data = {
        'step': [],
        'time': [],
        'P_log_frobenius': [],
        'mrp_error': [],
        'quat_error': [],
        'p_x': [], 'p_y': [], 'p_z': [],
        'q_w': [], 'q_x': [], 'q_y': [], 'q_z': [],
        'b_x': [], 'b_y': [], 'b_z': []
    }

    for line in data_lines:
        try:
            values = [float(x) for x in line.split(',')]
            if len(values) != 15:
                continue

            data['step'].append(values[0])
            data['time'].append(values[1])
            data['P_log_frobenius'].append(values[2])
            data['mrp_error'].append(values[3])
            data['quat_error'].append(values[4])
            data['p_x'].append(values[5])
            data['p_y'].append(values[6])
            data['p_z'].append(values[7])
            data['q_w'].append(values[8])
            data['q_x'].append(values[9])
            data['q_y'].append(values[10])
            data['q_z'].append(values[11])
            data['b_x'].append(values[12])
            data['b_y'].append(values[13])
            data['b_z'].append(values[14])
        except (ValueError, IndexError):
            continue

    # Convert to numpy arrays
    for key in data:
        data[key] = np.array(data[key])

    return data


def save_raw_log(lines, output_dir='data'):
    """Save raw log lines to file."""
    Path(output_dir).mkdir(exist_ok=True)
    output_path = Path(output_dir) / 'ekf_raw_log.txt'

    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))

    print(f"Raw log saved to: {output_path}")
    return output_path


def save_to_csv(data, output_dir='data'):
    """
    Save parsed data to CSV file.

    Args:
        data: Dictionary of numpy arrays from parse_ekf_log
        output_dir: Directory to save CSV file
    """
    # Create output directory if it doesn't exist
    Path(output_dir).mkdir(exist_ok=True)

    output_path = Path(output_dir) / 'ekf_convergence.csv'

    # Write CSV with header
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)

        # Write header
        writer.writerow([
            'step', 'time', 'P_log_frobenius', 'mrp_error', 'quat_error',
            'p_x', 'p_y', 'p_z', 'q_w', 'q_x', 'q_y', 'q_z',
            'b_x', 'b_y', 'b_z'
        ])

        # Write data rows
        for i in range(len(data['time'])):
            writer.writerow([
                int(data['step'][i]),
                data['time'][i],
                data['P_log_frobenius'][i],
                data['mrp_error'][i],
                data['quat_error'][i],
                data['p_x'][i],
                data['p_y'][i],
                data['p_z'][i],
                data['q_w'][i],
                data['q_x'][i],
                data['q_y'][i],
                data['q_z'][i],
                data['b_x'][i],
                data['b_y'][i],
                data['b_z'][i]
            ])

    print(f"CSV data saved to: {output_path}")
    return output_path


def plot_convergence(data, output_dir='plots'):
    """
    Generate convergence plots from EKF data.

    Args:
        data: Dictionary of numpy arrays from parse_ekf_log
        output_dir: Directory to save plots
    """
    # Create output directory if it doesn't exist
    Path(output_dir).mkdir(exist_ok=True)

    # Use time (seconds) for x-axis
    time = data['time']

    # Create figure with subplots
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('EKF Convergence Analysis (1M steps)', fontsize=16, fontweight='bold')

    # Plot 1: Log Frobenius Norm of Covariance Matrix P
    ax = axes[0, 0]
    ax.plot(time, data['P_log_frobenius'], 'b-', linewidth=1.5)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('log(||P||_F)', fontsize=11)
    ax.set_title('Covariance Matrix Log Frobenius Norm', fontweight='bold')
    ax.grid(True, alpha=0.3)

    # Plot 2: State Estimation Errors
    ax = axes[0, 1]
    ax.semilogy(time, data['mrp_error'], 'r-', label='MRP Error', linewidth=1.5)
    ax.semilogy(time, data['quat_error'], 'g-', label='Quaternion Error', linewidth=1.5, alpha=0.7)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Error Magnitude', fontsize=11)
    ax.set_title('Attitude Estimation Error', fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3, which='both')

    # Plot 3: MRP Components
    ax = axes[1, 0]
    ax.plot(time, data['p_x'], 'r-', label='p_x', linewidth=1.5)
    ax.plot(time, data['p_y'], 'g-', label='p_y', linewidth=1.5)
    ax.plot(time, data['p_z'], 'b-', label='p_z', linewidth=1.5)
    # Expected values for 90 deg rotation around z-axis
    ax.axhline(y=0.0, color='r', linestyle='--', alpha=0.5, label='p_x expected')
    ax.axhline(y=0.0, color='g', linestyle='--', alpha=0.5, label='p_y expected')
    ax.axhline(y=0.4142, color='b', linestyle='--', alpha=0.5, label='p_z expected (tan(π/8))')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('MRP Value', fontsize=11)
    ax.set_title('Modified Rodrigues Parameters', fontweight='bold')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, alpha=0.3)

    # Plot 4: Quaternion Components
    ax = axes[1, 1]
    ax.plot(time, data['q_w'], 'k-', label='q_w', linewidth=1.5)
    ax.plot(time, data['q_x'], 'r-', label='q_x', linewidth=1.5)
    ax.plot(time, data['q_y'], 'g-', label='q_y', linewidth=1.5)
    ax.plot(time, data['q_z'], 'b-', label='q_z', linewidth=1.5)
    # Expected values for 90 deg rotation around z-axis
    ax.axhline(y=0.7071, color='k', linestyle='--', alpha=0.5, label='q_w expected')
    ax.axhline(y=0.0, color='r', linestyle='--', alpha=0.5, label='q_x expected')
    ax.axhline(y=0.0, color='g', linestyle='--', alpha=0.5, label='q_y expected')
    ax.axhline(y=0.7071, color='b', linestyle='--', alpha=0.5, label='q_z expected')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Quaternion Value', fontsize=11)
    ax.set_title('Quaternion Representation', fontweight='bold')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, alpha=0.3)

    # Plot 5: Gyro Bias Estimates
    ax = axes[2, 0]
    ax.plot(time, data['b_x'], 'r-', label='b_x', linewidth=1.5)
    ax.plot(time, data['b_y'], 'g-', label='b_y', linewidth=1.5)
    ax.plot(time, data['b_z'], 'b-', label='b_z', linewidth=1.5)
    ax.axhline(y=0.0, color='k', linestyle='--', alpha=0.5, label='Expected (0)')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Bias (rad/s)', fontsize=11)
    ax.set_title('Gyroscope Bias Estimates', fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot 6: Combined Error and Covariance (for correlation analysis)
    ax = axes[2, 1]
    ax2 = ax.twinx()

    line1 = ax.semilogy(time, data['mrp_error'], 'r-', label='MRP Error', linewidth=1.5)
    line2 = ax2.plot(time, data['P_log_frobenius'], 'b-', label='log(||P||_F)', linewidth=1.5)

    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('MRP Error', fontsize=11, color='r')
    ax2.set_ylabel('log(||P||_F)', fontsize=11, color='b')
    ax.set_title('Error vs Covariance Correlation', fontweight='bold')
    ax.tick_params(axis='y', labelcolor='r')
    ax2.tick_params(axis='y', labelcolor='b')
    ax.grid(True, alpha=0.3, which='both')

    # Combine legends
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc='upper right')

    plt.tight_layout()

    # Save figure
    output_path = Path(output_dir) / 'ekf_convergence.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Plot saved to: {output_path}")

    # Show figure
    plt.show()

    # Print final statistics
    print("\n" + "="*60)
    print("CONVERGENCE STATISTICS")
    print("="*60)
    print(f"Total simulation time: {time[-1]:.1f} seconds")
    print(f"Total steps: {int(data['step'][-1]):,}")
    print(f"\nFinal State:")
    print(f"  MRP error: {data['mrp_error'][-1]:.6f}")
    print(f"  Quaternion error: {data['quat_error'][-1]:.6f}")
    print(f"  Log Frobenius norm: {data['P_log_frobenius'][-1]:.6f}")
    print(f"\nFinal Attitude (MRP): [{data['p_x'][-1]:.6f}, {data['p_y'][-1]:.6f}, {data['p_z'][-1]:.6f}]")
    print(f"Final Attitude (Quat): [{data['q_w'][-1]:.6f}, {data['q_x'][-1]:.6f}, {data['q_y'][-1]:.6f}, {data['q_z'][-1]:.6f}]")
    print(f"Final Gyro Bias: [{data['b_x'][-1]:.6f}, {data['b_y'][-1]:.6f}, {data['b_z'][-1]:.6f}] rad/s")
    print("="*60)


def main():
    """Main entry point."""
    port = sys.argv[1] if len(sys.argv) > 1 else None

    try:
        # Find and connect to serial port
        port = find_serial_port(port)

        # Read data from serial
        lines = read_from_serial(port)
        print(f"Read {len(lines)} lines from serial")

        # Save raw log
        save_raw_log(lines)

        # Parse log data
        data = parse_ekf_log(lines)
        print(f"Successfully parsed {len(data['time'])} data points")

        # Save to CSV
        save_to_csv(data)

        # Generate plots
        plot_convergence(data)

        print("\n✓ All done!")

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
