#!/usr/bin/env python3
"""
Colorized serial log viewer for satellite debugging.
Like tio, but with pretty colors based on log types.
"""

import serial
import argparse
from rich.console import Console
from rich.text import Text

# Color scheme for different log types
LOG_COLORS = {
    'magnetometer': 'cyan',
    'gps': 'green',
    'sun': 'yellow',
    'imu': 'magenta',
    'power': 'blue',
    'ekf': 'bright_red',
    'orbit': 'bright_cyan',
    'world': 'bright_yellow',
    'state': 'bright_blue',
    'test': 'bright_green',
    'error': 'red bold',
    'warning': 'yellow bold',
    'default': 'white'
}


def colorize_log(line):
    """Colorize a log line based on its content."""
    # Check for error/warning first
    if 'error' in line.lower() or 'err' in line.lower():
        color = LOG_COLORS['error']
    elif 'warn' in line.lower():
        color = LOG_COLORS['warning']
    # Determine log type and color based on tags
    elif '[sensor]' in line:
        if 'b_body' in line or 'Magnetometer' in line or 'mag' in line.lower():
            color = LOG_COLORS['magnetometer']
        elif 'Lat' in line or 'Lon' in line or 'GPS' in line or 'gps' in line.lower():
            color = LOG_COLORS['gps']
        elif 'sun' in line.lower():
            color = LOG_COLORS['sun']
        elif 'w_body' in line or 'IMU' in line or 'imu' in line.lower() or 'gyro' in line.lower():
            color = LOG_COLORS['imu']
        elif 'power' in line.lower() or 'voltage' in line.lower() or 'current' in line.lower():
            color = LOG_COLORS['power']
        elif 'a_body' in line:  # Acceleration data
            color = LOG_COLORS['imu']
        else:
            color = LOG_COLORS['default']
    elif '[ekf]' in line:
        color = LOG_COLORS['ekf']
    elif '[orbit]' in line:
        color = LOG_COLORS['orbit']
    elif '[world]' in line:
        color = LOG_COLORS['world']
    elif '[state]' in line:
        color = LOG_COLORS['state']
    elif '[test]' in line:
        color = LOG_COLORS['test']
    else:
        color = LOG_COLORS['default']

    # Create colored text
    text = Text(line, style=color)
    return text


def main():
    parser = argparse.ArgumentParser(
        description='Colorized serial log viewer for satellite debugging'
    )
    parser.add_argument(
        '--port',
        type=str,
        default='/dev/tty.usbmodem1101',
        help='Serial port (default: /dev/tty.usbmodem1101)'
    )
    parser.add_argument(
        '--baud',
        type=int,
        default=115200,
        help='Baud rate (default: 115200)'
    )

    args = parser.parse_args()

    console = Console()

    try:
        # Open serial connection
        console.print(f"[green]Opening serial port {args.port} at {args.baud} baud...[/green]")
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        console.print(f"[green]Connected! Press Ctrl+C to exit.[/green]\n")

        # Read and display logs with color
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    colored_text = colorize_log(line)
                    console.print(colored_text)

    except KeyboardInterrupt:
        console.print("\n[yellow]Shutting down...[/yellow]")
    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")
    finally:
        if 'ser' in locals() and ser:
            ser.close()
            console.print("[green]Serial port closed.[/green]")


if __name__ == '__main__':
    main()
