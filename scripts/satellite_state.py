#!/usr/bin/env python3
"""
Real-time satellite state visualization script.
Reads serial debug output and plots:
1. LLA world map
2. Body frame vectors (b_body, sun_vector_body)
3. ECI frame vectors (b_eci, sun_vector_eci)
4. Angular velocity over time
5. Attitude orientation (3 body axes relative to ECI)
6. Filter uncertainty (P_log_frobenius)
7. Gyro drift (b_gyro_drift)

Plus colorized terminal log output.
"""

import serial
import re
import numpy as np

# Don't force a specific backend - let matplotlib choose the best available one
# The performance gains from quiver reuse are much more important than backend choice
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import cartopy.crs as ccrs
import cartopy.feature as cfeature
from collections import deque
from rich.console import Console
from rich.text import Text
import argparse

# Performance optimizations
plt.rcParams['path.simplify'] = True
plt.rcParams['path.simplify_threshold'] = 1.0
plt.rcParams['agg.path.chunksize'] = 10000

# Configuration
UPDATE_FREQUENCY_HZ = 120  # Configurable update frequency
HISTORY_LENGTH = 100  # Number of points to keep for time series
ENABLE_TERMINAL_LOGS = False  # Disabled by default for maximum plotting performance

# Initialize rich console for colored output
console = Console()

# Color scheme for different log types
LOG_COLORS = {
    'magnetometer': 'cyan',
    'gps': 'green',
    'sun': 'yellow',
    'imu': 'magenta',
    'power': 'blue',
    'ekf': 'bright_red',
    'state': 'bright_blue',
    'default': 'white'
}


class SatelliteState:
    """Stores the current satellite state from parsed logs."""

    def __init__(self):
        # GPS/Position
        self.lat = None
        self.lon = None
        self.alt = None
        self.r_eci = None
        self.r_ecef = None

        # Magnetometer
        self.b_body = None
        self.b_body_raw = None
        self.b_eci = None

        # Sun sensors
        self.sun_vector_body = None
        self.sun_vector_eci = None

        # IMU
        self.w_body = deque(maxlen=HISTORY_LENGTH)
        self.w_body_current = None
        self.w_mag = None

        # Attitude
        self.q_eci_to_body = None  # Quaternion [x, y, z, w]
        self.p_eci_to_body = None
        self.b_gyro_drift = deque(maxlen=HISTORY_LENGTH)
        self.b_gyro_drift_current = None
        self.P_log_frobenius = deque(maxlen=HISTORY_LENGTH)
        self.P_log_frobenius_current = None

        # Cached quiver objects to avoid recreating
        self.body_b_quiver = None
        self.body_sun_quiver = None
        self.eci_b_quiver = None
        self.eci_sun_quiver = None
        self.att_x_quiver = None
        self.att_y_quiver = None
        self.att_z_quiver = None

        # Track if we need to update legends (only once)
        self.body_legend_set = False
        self.eci_legend_set = False
        self.att_legend_set = False


class LogParser:
    """Parses debug logs and extracts satellite state."""

    # Regex patterns for parsing logs (flexible whitespace)
    PATTERNS = {
        'lat_lon_alt': re.compile(r'\[sensor\]\s+Lat\s*=\s*([-\d.]+),\s*Lon\s*=\s*([-\d.]+),\s*Alt\s*=\s*([-\d.]+)'),
        'b_body': re.compile(r'\[sensor\]\s+b_body\s*=\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]'),
        'b_eci': re.compile(r'\[sensor\]\s+b_eci\s*=\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]'),
        'sun_vector_body': re.compile(r'\[sensor\]\s+sun_vector_body\s*=\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]'),
        'sun_vector_eci': re.compile(r'\[sensor\]\s+sun_vector_eci\s*=\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]'),
        'w_body': re.compile(r'\[sensor\]\s+w_body\s*=\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]'),
        'q_eci_to_body': re.compile(r'\[ekf\]\s+q_eci_to_body\s*=\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]'),
        'b_gyro_drift': re.compile(r'\[ekf\]\s+b_gyro_drift\s*=\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]'),
        'P_log_frobenius': re.compile(r'\[ekf\]\s+P_log_frobenius\s*=\s*([-\d.]+)'),
    }

    @staticmethod
    def parse_line(line, state):
        """Parse a single log line and update state."""
        # GPS position
        match = LogParser.PATTERNS['lat_lon_alt'].search(line)
        if match:
            state.lat = float(match.group(1))
            state.lon = float(match.group(2))
            state.alt = float(match.group(3))

        # Magnetometer body
        match = LogParser.PATTERNS['b_body'].search(line)
        if match:
            state.b_body = np.array([float(match.group(1)),
                                     float(match.group(2)),
                                     float(match.group(3))])

        # Magnetometer ECI
        match = LogParser.PATTERNS['b_eci'].search(line)
        if match:
            state.b_eci = np.array([float(match.group(1)),
                                    float(match.group(2)),
                                    float(match.group(3))])

        # Sun vector body
        match = LogParser.PATTERNS['sun_vector_body'].search(line)
        if match:
            state.sun_vector_body = np.array([float(match.group(1)),
                                              float(match.group(2)),
                                              float(match.group(3))])

        # Sun vector ECI
        match = LogParser.PATTERNS['sun_vector_eci'].search(line)
        if match:
            state.sun_vector_eci = np.array([float(match.group(1)),
                                             float(match.group(2)),
                                             float(match.group(3))])

        # Angular velocity
        match = LogParser.PATTERNS['w_body'].search(line)
        if match:
            state.w_body_current = np.array([float(match.group(1)),
                                             float(match.group(2)),
                                             float(match.group(3))])
            state.w_body.append(state.w_body_current)

        # Quaternion
        match = LogParser.PATTERNS['q_eci_to_body'].search(line)
        if match:
            state.q_eci_to_body = np.array([float(match.group(1)),
                                            float(match.group(2)),
                                            float(match.group(3)),
                                            float(match.group(4))])

        # Gyro drift
        match = LogParser.PATTERNS['b_gyro_drift'].search(line)
        if match:
            state.b_gyro_drift_current = np.array([float(match.group(1)),
                                                   float(match.group(2)),
                                                   float(match.group(3))])
            state.b_gyro_drift.append(state.b_gyro_drift_current)

        # Covariance
        match = LogParser.PATTERNS['P_log_frobenius'].search(line)
        if match:
            state.P_log_frobenius_current = float(match.group(1))
            state.P_log_frobenius.append(state.P_log_frobenius_current)

    @staticmethod
    def colorize_log(line):
        """Colorize a log line based on its content."""
        # Determine log type and color
        if '[sensor]' in line:
            if 'b_body' in line or 'Magnetometer' in line:
                color = LOG_COLORS['magnetometer']
            elif 'Lat' in line or 'Lon' in line or 'GPS' in line:
                color = LOG_COLORS['gps']
            elif 'sun' in line.lower():
                color = LOG_COLORS['sun']
            elif 'w_body' in line or 'IMU' in line:
                color = LOG_COLORS['imu']
            elif 'power' in line.lower() or 'voltage' in line.lower():
                color = LOG_COLORS['power']
            else:
                color = LOG_COLORS['default']
        elif '[ekf]' in line:
            color = LOG_COLORS['ekf']
        elif '[state]' in line:
            color = LOG_COLORS['state']
        else:
            color = LOG_COLORS['default']

        # Create colored text
        text = Text(line, style=color)
        return text


def quaternion_to_rotation_matrix(q):
    """Convert quaternion [x, y, z, w] to rotation matrix."""
    if q is None:
        return np.eye(3)

    x, y, z, w = q

    # Rotation matrix from quaternion
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])

    return R


class SatellitePlotter:
    """Real-time plotter for satellite state."""

    def __init__(self, serial_port, baud_rate, update_freq):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.update_interval = 1000 / update_freq  # milliseconds
        self.state = SatelliteState()
        self.ser = None

        # Setup figure and subplots
        self.fig = plt.figure(figsize=(18, 12))
        self.setup_plots()

    def setup_plots(self):
        """Initialize all subplot axes."""
        # 1. LLA World Map (top left) - simplified for speed
        try:
            self.ax_map = self.fig.add_subplot(3, 3, 1, projection=ccrs.PlateCarree())
            self.ax_map.set_global()
            # Only add coastline for speed - skip ocean/land fills
            self.ax_map.add_feature(cfeature.COASTLINE, linewidth=0.5, edgecolor='black')
            self.ax_map.gridlines(draw_labels=False, alpha=0.3)  # Disable labels for speed
            self.ax_map.set_title('GPS Position (LLA)')
            self.map_point, = self.ax_map.plot([], [], 'ro', markersize=8, transform=ccrs.PlateCarree())
            self.map_text = self.ax_map.text(0.02, 0.98, '', transform=self.ax_map.transAxes,
                                             verticalalignment='top', fontsize=8,
                                             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
            self.use_cartopy = True
        except:
            # Fallback to simple 2D plot if cartopy fails
            self.ax_map = self.fig.add_subplot(3, 3, 1)
            self.ax_map.set_xlim(-180, 180)
            self.ax_map.set_ylim(-90, 90)
            self.ax_map.set_xlabel('Longitude')
            self.ax_map.set_ylabel('Latitude')
            self.ax_map.set_title('GPS Position (LLA)')
            self.ax_map.grid(True, alpha=0.3)
            self.map_point, = self.ax_map.plot([], [], 'ro', markersize=8)
            self.map_text = self.ax_map.text(0.02, 0.98, '', transform=self.ax_map.transAxes,
                                             verticalalignment='top', fontsize=8,
                                             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
            self.use_cartopy = False

        # 2. Body Frame Vectors (top middle)
        self.ax_body = self.fig.add_subplot(3, 3, 2, projection='3d')
        self.ax_body.set_title('Body Frame Vectors')
        self.ax_body.set_xlabel('X')
        self.ax_body.set_ylabel('Y')
        self.ax_body.set_zlabel('Z')
        self.ax_body.set_xlim([-1, 1])
        self.ax_body.set_ylim([-1, 1])
        self.ax_body.set_zlim([-1, 1])

        # 3. ECI Frame Vectors (top right)
        self.ax_eci = self.fig.add_subplot(3, 3, 3, projection='3d')
        self.ax_eci.set_title('ECI Frame Vectors')
        self.ax_eci.set_xlabel('X')
        self.ax_eci.set_ylabel('Y')
        self.ax_eci.set_zlabel('Z')
        self.ax_eci.set_xlim([-1, 1])
        self.ax_eci.set_ylim([-1, 1])
        self.ax_eci.set_zlim([-1, 1])

        # 4. Angular Velocity (middle left)
        self.ax_w = self.fig.add_subplot(3, 3, 4)
        self.ax_w.set_title('Angular Velocity (Body Frame)')
        self.ax_w.set_xlabel('Sample')
        self.ax_w.set_ylabel('ω (rad/s)')
        self.ax_w.grid(True, alpha=0.3)
        self.line_wx, = self.ax_w.plot([], [], 'r-', label='ωx', linewidth=1.5)
        self.line_wy, = self.ax_w.plot([], [], 'g-', label='ωy', linewidth=1.5)
        self.line_wz, = self.ax_w.plot([], [], 'b-', label='ωz', linewidth=1.5)
        self.ax_w.legend()

        # 5. Attitude Orientation (middle middle)
        self.ax_att = self.fig.add_subplot(3, 3, 5, projection='3d')
        self.ax_att.set_title('Attitude (Body Frame in ECI)')
        self.ax_att.set_xlabel('ECI X')
        self.ax_att.set_ylabel('ECI Y')
        self.ax_att.set_zlabel('ECI Z')
        self.ax_att.set_xlim([-1, 1])
        self.ax_att.set_ylim([-1, 1])
        self.ax_att.set_zlim([-1, 1])

        # 6. Filter Uncertainty (middle right)
        self.ax_cov = self.fig.add_subplot(3, 3, 6)
        self.ax_cov.set_title('Filter Uncertainty (Log Frobenius Norm)')
        self.ax_cov.set_xlabel('Sample')
        self.ax_cov.set_ylabel('log(||P||_F)')
        self.ax_cov.grid(True, alpha=0.3)
        self.line_cov, = self.ax_cov.plot([], [], 'purple', linewidth=2)

        # 7. Gyro Drift (bottom left, spanning 3 columns)
        self.ax_drift = self.fig.add_subplot(3, 3, (7, 9))
        self.ax_drift.set_title('Gyro Drift (Body Frame)')
        self.ax_drift.set_xlabel('Sample')
        self.ax_drift.set_ylabel('b_gyro (rad/s)')
        self.ax_drift.grid(True, alpha=0.3)
        self.line_drift_x, = self.ax_drift.plot([], [], 'r-', label='drift_x', linewidth=1.5)
        self.line_drift_y, = self.ax_drift.plot([], [], 'g-', label='drift_y', linewidth=1.5)
        self.line_drift_z, = self.ax_drift.plot([], [], 'b-', label='drift_z', linewidth=1.5)
        self.ax_drift.legend()

        plt.tight_layout()

    def update_map(self):
        """Update GPS position on map."""
        if self.state.lat is not None and self.state.lon is not None:
            self.map_point.set_data([self.state.lon], [self.state.lat])
            text = f"Lat: {self.state.lat:.6f}°\nLon: {self.state.lon:.6f}°\nAlt: {self.state.alt:.3f} km"
            self.map_text.set_text(text)

    def update_body_vectors(self):
        """Update body frame vector plot - for 3D quivers, update segments directly."""
        origin = np.array([0, 0, 0])

        # Initialize or update b_body quiver
        if self.state.b_body is not None:
            if self.state.body_b_quiver is None:
                self.state.body_b_quiver = self.ax_body.quiver(*origin, *self.state.b_body,
                                   color='purple', arrow_length_ratio=0.15, linewidth=2,
                                   label='b_body')
            else:
                # Fast update for 3D quiver: modify segment data directly
                segments = [[[0, 0, 0], self.state.b_body.tolist()]]
                self.state.body_b_quiver.set_segments(segments)

        # Initialize or update sun_vector_body quiver
        if self.state.sun_vector_body is not None:
            if self.state.body_sun_quiver is None:
                self.state.body_sun_quiver = self.ax_body.quiver(*origin, *self.state.sun_vector_body,
                                   color='pink', arrow_length_ratio=0.15, linewidth=2,
                                   label='sun_body')
            else:
                # Fast update for 3D quiver: modify segment data directly
                segments = [[[0, 0, 0], self.state.sun_vector_body.tolist()]]
                self.state.body_sun_quiver.set_segments(segments)

        # Only set legend once
        if not self.state.body_legend_set and (self.state.b_body is not None or self.state.sun_vector_body is not None):
            self.ax_body.legend()
            self.state.body_legend_set = True

    def update_eci_vectors(self):
        """Update ECI frame vector plot - for 3D quivers, update segments directly."""
        origin = np.array([0, 0, 0])

        # Initialize or update b_eci quiver
        if self.state.b_eci is not None:
            if self.state.eci_b_quiver is None:
                self.state.eci_b_quiver = self.ax_eci.quiver(*origin, *self.state.b_eci,
                                  color='cyan', arrow_length_ratio=0.15, linewidth=2,
                                  label='b_eci')
            else:
                # Fast update for 3D quiver: modify segment data directly
                segments = [[[0, 0, 0], self.state.b_eci.tolist()]]
                self.state.eci_b_quiver.set_segments(segments)

        # Initialize or update sun_vector_eci quiver
        if self.state.sun_vector_eci is not None:
            if self.state.eci_sun_quiver is None:
                self.state.eci_sun_quiver = self.ax_eci.quiver(*origin, *self.state.sun_vector_eci,
                                  color='yellow', arrow_length_ratio=0.15, linewidth=2,
                                  label='sun_eci')
            else:
                # Fast update for 3D quiver: modify segment data directly
                segments = [[[0, 0, 0], self.state.sun_vector_eci.tolist()]]
                self.state.eci_sun_quiver.set_segments(segments)

        # Only set legend once
        if not self.state.eci_legend_set and (self.state.b_eci is not None or self.state.sun_vector_eci is not None):
            self.ax_eci.legend()
            self.state.eci_legend_set = True

    def update_angular_velocity(self):
        """Update angular velocity time series."""
        if len(self.state.w_body) > 0:
            w_array = np.array(self.state.w_body)
            x_data = np.arange(len(w_array))

            self.line_wx.set_data(x_data, w_array[:, 0])
            self.line_wy.set_data(x_data, w_array[:, 1])
            self.line_wz.set_data(x_data, w_array[:, 2])

            # Only update limits if deque is full or data extends beyond current limits
            if len(w_array) == HISTORY_LENGTH or len(w_array) == 1:
                self.ax_w.set_xlim(0, max(HISTORY_LENGTH, len(w_array)))
                y_min = np.min(w_array) - 0.1
                y_max = np.max(w_array) + 0.1
                self.ax_w.set_ylim(y_min, y_max)

    def update_attitude(self):
        """Update attitude orientation visualization - for 3D quivers, update segments directly."""
        if self.state.q_eci_to_body is not None:
            # Get rotation matrix from quaternion
            R = quaternion_to_rotation_matrix(self.state.q_eci_to_body)

            # Body frame axes (identity in body frame)
            body_x = np.array([1, 0, 0])
            body_y = np.array([0, 1, 0])
            body_z = np.array([0, 0, 1])

            # Transform to ECI frame (inverse rotation since q is ECI to body)
            # R transforms from body to ECI, so R^T transforms ECI to body
            # We want body axes in ECI, so we use R
            eci_x = R @ body_x
            eci_y = R @ body_y
            eci_z = R @ body_z

            origin = np.array([0, 0, 0])

            # Initialize quivers on first run
            if self.state.att_x_quiver is None:
                self.state.att_x_quiver = self.ax_att.quiver(*origin, *eci_x, color='red',
                                  arrow_length_ratio=0.15, linewidth=2.5, label='Body X')
                self.state.att_y_quiver = self.ax_att.quiver(*origin, *eci_y, color='green',
                                  arrow_length_ratio=0.15, linewidth=2.5, label='Body Y')
                self.state.att_z_quiver = self.ax_att.quiver(*origin, *eci_z, color='blue',
                                  arrow_length_ratio=0.15, linewidth=2.5, label='Body Z')
            else:
                # Fast update for 3D quiver: modify segment data directly
                self.state.att_x_quiver.set_segments([[[0, 0, 0], eci_x.tolist()]])
                self.state.att_y_quiver.set_segments([[[0, 0, 0], eci_y.tolist()]])
                self.state.att_z_quiver.set_segments([[[0, 0, 0], eci_z.tolist()]])

            # Only set legend once
            if not self.state.att_legend_set:
                self.ax_att.legend()
                self.state.att_legend_set = True

    def update_covariance(self):
        """Update filter uncertainty plot."""
        if len(self.state.P_log_frobenius) > 0:
            x_data = np.arange(len(self.state.P_log_frobenius))
            y_data = list(self.state.P_log_frobenius)

            self.line_cov.set_data(x_data, y_data)

            # Only update limits if deque is full or first data point
            if len(y_data) == HISTORY_LENGTH or len(y_data) == 1:
                self.ax_cov.set_xlim(0, max(HISTORY_LENGTH, len(y_data)))
                y_min = min(y_data) - 0.5
                y_max = max(y_data) + 0.5
                self.ax_cov.set_ylim(y_min, y_max)

    def update_gyro_drift(self):
        """Update gyro drift time series."""
        if len(self.state.b_gyro_drift) > 0:
            drift_array = np.array(self.state.b_gyro_drift)
            x_data = np.arange(len(drift_array))

            self.line_drift_x.set_data(x_data, drift_array[:, 0])
            self.line_drift_y.set_data(x_data, drift_array[:, 1])
            self.line_drift_z.set_data(x_data, drift_array[:, 2])

            # Only update limits if deque is full or data extends beyond current limits
            if len(drift_array) == HISTORY_LENGTH or len(drift_array) == 1:
                self.ax_drift.set_xlim(0, max(HISTORY_LENGTH, len(drift_array)))
                y_min = np.min(drift_array) - 0.001
                y_max = np.max(drift_array) + 0.001
                self.ax_drift.set_ylim(y_min, y_max)

    def animate(self, frame):
        """Animation update function."""
        # Read and parse ALL available serial data (to stay current)
        lines_processed = 0
        max_lines_per_frame = 100  # Process max 100 lines per frame (increased from 50)
        print_lines = []  # Batch print lines for better performance

        if self.ser and self.ser.in_waiting > 0:
            try:
                while self.ser.in_waiting > 0 and lines_processed < max_lines_per_frame:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # Parse the line
                        LogParser.parse_line(line, self.state)

                        # Collect lines for batch printing (only first 5 to reduce overhead)
                        if lines_processed < 5:
                            print_lines.append(line)

                        lines_processed += 1

                # Batch print collected lines (more efficient than printing one at a time)
                if ENABLE_TERMINAL_LOGS and print_lines:
                    for line in print_lines:
                        colored_text = LogParser.colorize_log(line)
                        console.print(colored_text, end='')

                # If we skipped printing some lines, indicate that
                if ENABLE_TERMINAL_LOGS and lines_processed > 5:
                    console.print(f"[dim]... ({lines_processed - 5} more lines processed)[/dim]")

            except Exception as e:
                console.print(f"[red]Error reading serial: {e}[/red]")

        # Update all plots (using latest values only)
        self.update_map()
        self.update_body_vectors()
        self.update_eci_vectors()
        self.update_angular_velocity()
        self.update_attitude()
        self.update_covariance()
        self.update_gyro_drift()

        return []

    def run(self):
        """Start the real-time plotting."""
        try:
            # Open serial connection
            console.print(f"[green]Opening serial port {self.serial_port} at {self.baud_rate} baud...[/green]")
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            console.print(f"[green]Connected! Update frequency: {UPDATE_FREQUENCY_HZ} Hz[/green]")
            console.print("[yellow]Starting real-time visualization...[/yellow]\n")

            # Start animation
            anim = FuncAnimation(self.fig, self.animate, interval=self.update_interval,
                               blit=False, cache_frame_data=False)
            plt.show()

        except KeyboardInterrupt:
            console.print("\n[yellow]Shutting down...[/yellow]")
        except Exception as e:
            console.print(f"[red]Error: {e}[/red]")
        finally:
            if self.ser:
                self.ser.close()
                console.print("[green]Serial port closed.[/green]")


def main():
    parser = argparse.ArgumentParser(description='Real-time satellite state visualization')
    parser.add_argument('--port', type=str, default='/dev/tty.usbmodem1101',
                       help='Serial port (default: /dev/tty.usbmodem1101)')
    parser.add_argument('--baud', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('--freq', type=int, default=10,
                       help=f'Update frequency in Hz (default: 10)')
    parser.add_argument('--logs', action='store_true',
                       help='Enable terminal log output (disabled by default for performance)')

    args = parser.parse_args()

    # Update global config based on args
    global ENABLE_TERMINAL_LOGS
    ENABLE_TERMINAL_LOGS = args.logs

    # Create and run plotter
    plotter = SatellitePlotter(args.port, args.baud, args.freq)
    plotter.run()


if __name__ == '__main__':
    main()