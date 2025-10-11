#!/usr/bin/env python3
"""
Simple test to verify packet structure matches between Python and C.

This prints the expected packet sizes and structure without requiring
a connected device.
"""

import struct
import numpy as np


def test_sensor_packet():
    """Test sensor packet structure."""
    print("=" * 60)
    print("SENSOR PACKET STRUCTURE TEST")
    print("=" * 60)

    # Header
    header = b'SS'
    print(f"Header: {header} ({len(header)} bytes)")

    # IMU data (3 floats)
    w_body = struct.pack('<3f', 0.1, 0.2, 0.3)
    print(f"IMU (w_body): 3 floats = {len(w_body)} bytes")

    # Magnetometer data (3 floats)
    b_field = struct.pack('<3f', 0.7, 0.6, 0.5)
    print(f"Magnetometer (b_field): 3 floats = {len(b_field)} bytes")

    # GPS data (3 floats)
    gps = struct.pack('<3f', 37.4, -122.1, 12345.0)
    print(f"GPS (lat, lon, time): 3 floats = {len(gps)} bytes")

    # Sun sensors (16 uint16)
    sun_sensors = struct.pack('<16H', *range(16))
    print(f"Sun sensors: 16 uint16 = {len(sun_sensors)} bytes")

    # Assemble packet (without checksum)
    packet_data = w_body + b_field + gps + sun_sensors
    packet = header + packet_data

    # Checksum (2 bytes)
    checksum = sum(packet) & 0xFFFF
    checksum_bytes = struct.pack('<H', checksum)
    print(f"Checksum: uint16 = {len(checksum_bytes)} bytes")

    # Final packet
    full_packet = packet + checksum_bytes

    print(f"\nTotal packet size: {len(full_packet)} bytes")
    print(f"  Header:        {len(header)} bytes")
    print(f"  IMU:           {len(w_body)} bytes")
    print(f"  Magnetometer:  {len(b_field)} bytes")
    print(f"  GPS:           {len(gps)} bytes")
    print(f"  Sun sensors:   {len(sun_sensors)} bytes")
    print(f"  Checksum:      {len(checksum_bytes)} bytes")

    # Verify checksum
    received_checksum = struct.unpack('<H', full_packet[-2:])[0]
    calculated_checksum = sum(full_packet[:-2]) & 0xFFFF
    print(f"\nChecksum verification:")
    print(f"  Calculated: 0x{calculated_checksum:04X}")
    print(f"  Received:   0x{received_checksum:04X}")
    print(f"  Match: {calculated_checksum == received_checksum}")

    # Show hex dump of first few bytes
    print(f"\nFirst 32 bytes (hex):")
    print(' '.join(f'{b:02X}' for b in full_packet[:32]))

    return full_packet


def test_actuator_packet():
    """Test actuator packet structure."""
    print("\n" + "=" * 60)
    print("ACTUATOR PACKET STRUCTURE TEST")
    print("=" * 60)

    # Header
    header = b'AA'
    print(f"Header: {header} ({len(header)} bytes)")

    # Magnetorquer commands (3 floats)
    magdrv = struct.pack('<3f', -0.5, 0.3, 0.8)
    print(f"Magnetorquers (magdrv): 3 floats = {len(magdrv)} bytes")

    # Reaction wheel commands (3 floats)
    rw_speeds = struct.pack('<3f', 1.2, -0.4, 0.0)
    print(f"Reaction wheels: 3 floats = {len(rw_speeds)} bytes")

    # Assemble packet (without checksum)
    packet_data = magdrv + rw_speeds
    packet = header + packet_data

    # Checksum (2 bytes)
    checksum = sum(packet) & 0xFFFF
    checksum_bytes = struct.pack('<H', checksum)
    print(f"Checksum: uint16 = {len(checksum_bytes)} bytes")

    # Final packet
    full_packet = packet + checksum_bytes

    print(f"\nTotal packet size: {len(full_packet)} bytes")
    print(f"  Header:          {len(header)} bytes")
    print(f"  Magnetorquers:   {len(magdrv)} bytes")
    print(f"  Reaction wheels: {len(rw_speeds)} bytes")
    print(f"  Checksum:        {len(checksum_bytes)} bytes")

    # Verify checksum
    received_checksum = struct.unpack('<H', full_packet[-2:])[0]
    calculated_checksum = sum(full_packet[:-2]) & 0xFFFF
    print(f"\nChecksum verification:")
    print(f"  Calculated: 0x{calculated_checksum:04X}")
    print(f"  Received:   0x{received_checksum:04X}")
    print(f"  Match: {calculated_checksum == received_checksum}")

    # Show hex dump
    print(f"\nFull packet (hex):")
    print(' '.join(f'{b:02X}' for b in full_packet))

    return full_packet


def test_c_struct_sizes():
    """Calculate expected C struct sizes."""
    print("\n" + "=" * 60)
    print("EXPECTED C STRUCT SIZES")
    print("=" * 60)

    # sensor packet
    sensor_size = (
        2 +      # header (2 bytes)
        3 * 4 +  # IMU (3 floats)
        3 * 4 +  # Magnetometer (3 floats)
        3 * 4 +  # GPS (3 floats)
        16 * 2 + # Sun sensors (16 uint16)
        2        # checksum (uint16)
    )
    print(f"sim_sensor_packet_t: {sensor_size} bytes")

    # actuator packet
    actuator_size = (
        2 +      # header (2 bytes)
        3 * 4 +  # Magnetorquers (3 floats)
        3 * 4 +  # Reaction wheels (3 floats)
        2        # checksum (uint16)
    )
    print(f"sim_actuator_packet_t: {actuator_size} bytes")


if __name__ == '__main__':
    test_c_struct_sizes()
    sensor_pkt = test_sensor_packet()
    actuator_pkt = test_actuator_packet()

    print("\n" + "=" * 60)
    print("TEST COMPLETE")
    print("=" * 60)
    print("\nIf running in SIMULATION mode, the flight computer should print:")
    print(f"  Sensor packet size: {len(sensor_pkt)} bytes")
    print(f"  Actuator packet size: {len(actuator_pkt)} bytes")
