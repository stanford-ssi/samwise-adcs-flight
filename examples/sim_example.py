#!/usr/bin/env python3
"""
Example simulation script for SAMWISE ADCS hardware-in-the-loop testing.

This script sends fake sensor data to the flight computer via USB serial
and receives actuator commands back. Useful for testing GNC algorithms
without physical hardware.

Hardware setup:
1. Flash RP2350B with SIMULATION mode firmware
2. Connect via USB
3. Run this script

The flight computer will run at whatever rate you send packets.
"""

import serial
import struct
import time
import numpy as np
import argparse


class ADCSSimulator:
    """Simple ADCS simulator for hardware-in-the-loop testing."""

    # Packet headers
    SENSOR_HEADER = b'SS'
    ACTUATOR_HEADER = b'AA'

    def __init__(self, port='/dev/tty.usbmodem1234l', baudrate=115200):
        """Initialize serial connection to flight computer."""
        self.ser = serial.Serial(port, baudrate, timeout=1.0)
        print(f"Connected to {port} at {baudrate} baud")

        # Simulation state
        self.time = 0.0
        self.dt = 0.1  # 10Hz control loop

        # Orbital parameters (simple polar orbit)
        self.orbital_period = 90.0 * 60.0  # 90 minutes in seconds
        self.orbital_freq = 2.0 * np.pi / self.orbital_period

        # Last received actuator commands
        self.magdrv_requested = np.array([0.0, 0.0, 0.0])
        self.rw_speeds = np.array([0.0, 0.0, 0.0])

    def calculate_checksum(self, data):
        """Calculate simple checksum (sum of all bytes)."""
        return sum(data) & 0xFFFF

    def compute_magnetic_field(self):
        """
        Compute magnetic field in body frame.

        For a polar orbit, the magnetic field rotates as the satellite
        moves around Earth. Simplified model: sinusoidal variation.

        Returns:
            np.array: Magnetic field unit vector [Bx, By, Bz]
        """
        # Simple sinusoidal model (not physically accurate, but good for testing)
        angle = self.orbital_freq * self.time

        Bx = 0.6 * np.cos(angle)
        By = 0.5 * np.sin(angle)
        Bz = 0.6 * np.sin(angle * 2)  # Higher frequency variation

        # Normalize to unit vector
        B = np.array([Bx, By, Bz])
        B_norm = B / np.linalg.norm(B)

        return B_norm

    def compute_angular_velocity(self):
        """
        Compute angular velocity in body frame.

        For testing, start with some initial tumble that decays.

        Returns:
            np.array: Angular velocity [wx, wy, wz] in rad/s
        """
        # Initial tumble rate (deg/s) that decays over time
        decay_factor = np.exp(-self.time / 100.0)

        wx = 5.0 * np.deg2rad(1.0) * decay_factor * np.sin(0.5 * self.time)
        wy = 3.0 * np.deg2rad(1.0) * decay_factor * np.cos(0.3 * self.time)
        wz = 2.0 * np.deg2rad(1.0) * decay_factor * np.sin(0.7 * self.time)

        return np.array([wx, wy, wz])

    def compute_gps_position(self):
        """
        Compute GPS position (lat, lon) for polar orbit.

        Returns:
            tuple: (latitude, longitude, gps_time) in degrees
        """
        # Simple polar orbit - latitude varies sinusoidally
        lat = 60.0 * np.sin(self.orbital_freq * self.time)
        lon = (self.time * 360.0 / self.orbital_period) % 360.0 - 180.0
        gps_time = self.time

        return lat, lon, gps_time

    def compute_sun_sensors(self):
        """
        Compute sun sensor readings.

        Simplified: assume sun illumination varies with orbital position.

        Returns:
            np.array: 16 sun sensor ADC values [0-4095]
        """
        # Simplified sun vector in body frame
        sun_angle = self.orbital_freq * self.time
        sun_vector = np.array([
            np.cos(sun_angle),
            np.sin(sun_angle),
            0.3
        ])
        sun_vector /= np.linalg.norm(sun_vector)

        # Sensor normals (simplified - just use 16 random directions)
        # In reality these would match the actual sensor orientations
        sensors = np.zeros(16, dtype=np.uint16)

        for i in range(16):
            # Create a sensor normal vector
            theta = 2.0 * np.pi * i / 16.0
            phi = np.pi / 4.0

            normal = np.array([
                np.sin(phi) * np.cos(theta),
                np.sin(phi) * np.sin(theta),
                np.cos(phi)
            ])

            # Dot product gives illumination intensity
            intensity = max(0.0, np.dot(sun_vector, normal))

            # Convert to 12-bit ADC value (0-4095)
            sensors[i] = int(intensity * 4095.0)

        return sensors

    def pack_sensor_packet(self):
        """
        Pack sensor data into binary packet for flight computer.

        Packet structure (little-endian):
        - Header: 2 bytes ('SS')
        - IMU: 3 floats (12 bytes)
        - Magnetometer: 3 floats (12 bytes)
        - GPS: 3 floats (12 bytes)
        - Sun sensors: 16 uint16 (32 bytes)
        - Checksum: uint16 (2 bytes)
        Total: 72 bytes
        """
        # Compute sensor values
        w_body = self.compute_angular_velocity()
        b_field = self.compute_magnetic_field()
        gps_lat, gps_lon, gps_time = self.compute_gps_position()
        sun_sensors = self.compute_sun_sensors()

        # Pack into binary format
        packet_data = struct.pack(
            '<3f3f3f16H',  # Little-endian: 3 floats + 3 floats + 3 floats + 16 uint16
            w_body[0], w_body[1], w_body[2],
            b_field[0], b_field[1], b_field[2],
            gps_lat, gps_lon, gps_time,
            *sun_sensors
        )

        # Add header
        packet = self.SENSOR_HEADER + packet_data

        # Calculate and add checksum
        checksum = self.calculate_checksum(packet)
        packet += struct.pack('<H', checksum)

        return packet, w_body, b_field

    def read_actuator_packet(self, verbose=False):
        """
        Read actuator packet from serial, filtering out debug text.

        Returns:
            bytes: Actuator packet or empty bytes if timeout
        """
        packet = b''
        timeout = time.time() + 2.0  # 2 second timeout
        skipped_lines = []

        while len(packet) < 28 and time.time() < timeout:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)

                # Check if this looks like the start of a text line
                if len(packet) == 0 and byte in (b'[', b'\r', b'\n', b' '):
                    # Skip text output - read until newline
                    line = byte
                    while byte != b'\n' and self.ser.in_waiting > 0:
                        byte = self.ser.read(1)
                        line += byte

                    if verbose and line.strip():
                        skipped_lines.append(line.decode('utf-8', errors='replace').strip())
                    continue

                packet += byte

        if verbose and skipped_lines:
            print(f"  [DEBUG] Skipped {len(skipped_lines)} text lines from flight computer")
            for line in skipped_lines[:3]:  # Show first 3
                print(f"    {line}")

        return packet

    def unpack_actuator_packet(self, packet):
        """
        Unpack actuator command packet from flight computer.

        Packet structure (little-endian):
        - Header: 2 bytes ('AA')
        - Magnetorquers: 3 floats (12 bytes)
        - Reaction wheels: 3 floats (12 bytes)
        - Checksum: uint16 (2 bytes)
        Total: 28 bytes
        """
        if len(packet) != 28:
            return False

        # Verify header
        if packet[:2] != self.ACTUATOR_HEADER:
            return False

        # Verify checksum
        received_checksum = struct.unpack('<H', packet[-2:])[0]
        calculated_checksum = self.calculate_checksum(packet[:-2])

        if received_checksum != calculated_checksum:
            print(f"Warning: Checksum mismatch! Expected {calculated_checksum:04X}, got {received_checksum:04X}")
            return False

        # Unpack data
        data = struct.unpack('<3f3f', packet[2:-2])
        self.magdrv_requested = np.array(data[0:3])
        self.rw_speeds = np.array(data[3:6])

        return True

    def run(self, duration=60.0, speed_multiplier=1.0):
        """
        Run simulation loop.

        Args:
            duration: Simulation duration in seconds
            speed_multiplier: Speed up factor (1.0 = real-time, 10.0 = 10x faster)
        """
        print(f"\nStarting simulation for {duration}s at {speed_multiplier}x speed")
        print(f"Control loop rate: {1.0/self.dt} Hz")
        print("\nWaiting for flight computer to initialize...")
        time.sleep(7.0)  # Wait for flight computer boot

        start_time = time.time()
        iteration = 0

        try:
            while self.time < duration:
                iter_start = time.time()

                # Pack and send sensor packet
                sensor_packet, w_body, b_field = self.pack_sensor_packet()

                verbose = (iteration % 10 == 0)  # Status every 10 iterations

                self.ser.write(sensor_packet)

                # Wait for actuator response (with text filtering)
                actuator_packet = self.read_actuator_packet(verbose=False)

                if len(actuator_packet) == 28:
                    if self.unpack_actuator_packet(actuator_packet):
                        if verbose:
                            w_mag = np.linalg.norm(w_body)
                            print(f"\n[t={self.time:6.1f}s] Sim Status:")
                            print(f"  w_body: [{w_body[0]:+.4f}, {w_body[1]:+.4f}, {w_body[2]:+.4f}] rad/s (|w| = {w_mag:.4f} rad/s = {np.rad2deg(w_mag):.2f} deg/s)")
                            print(f"  B_body: [{b_field[0]:+.4f}, {b_field[1]:+.4f}, {b_field[2]:+.4f}]")
                            print(f"  M_cmd:  [{self.magdrv_requested[0]:+.4f}, {self.magdrv_requested[1]:+.4f}, {self.magdrv_requested[2]:+.4f}]")
                            print(f"  RW_cmd: [{self.rw_speeds[0]:+.4f}, {self.rw_speeds[1]:+.4f}, {self.rw_speeds[2]:+.4f}] rad/s")

                # Advance time
                self.time += self.dt
                iteration += 1

                # Rate limiting based on speed multiplier
                elapsed = time.time() - iter_start
                sleep_time = (self.dt / speed_multiplier) - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\nSimulation interrupted by user")

        finally:
            elapsed_real = time.time() - start_time
            print(f"\n=== Simulation Complete ===")
            print(f"Simulated time: {self.time:.1f}s")
            print(f"Real time: {elapsed_real:.1f}s")
            print(f"Speed: {self.time/elapsed_real:.1f}x real-time")
            print(f"Iterations: {iteration}")
            self.ser.close()


def main():
    parser = argparse.ArgumentParser(description='ADCS Hardware-in-Loop Simulator')
    parser.add_argument('--port', type=str, default='/dev/tty.usbmodem1234',
                        help='Serial port (default: /dev/tty.usbmodem1234)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--duration', type=float, default=60.0,
                        help='Simulation duration in seconds (default: 60)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Speed multiplier (default: 1.0 = real-time)')

    args = parser.parse_args()

    print("=" * 60)
    print("SAMWISE ADCS Hardware-in-Loop Simulator")
    print("=" * 60)

    sim = ADCSSimulator(port=args.port, baudrate=args.baud)
    sim.run(duration=args.duration, speed_multiplier=args.speed)


if __name__ == '__main__':
    main()
