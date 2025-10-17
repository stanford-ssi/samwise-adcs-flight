#!/usr/bin/env julia
"""
ADCS Simulation Interface Driver

This module provides a clean driver interface for communicating with the ADCS
flight software via serial. Designed to be integrated into physics-based simulations.

# Core Driver API:
- `ADCSDriver`: Stateful driver object managing serial connection
- `send_sensor_packet!(driver, sensor_data)`: Send sensor data to ADCS
- `receive_actuator_packet(driver)`: Receive actuator commands from ADCS
- `wait_for_ready(driver)`: Wait for ADCS initialization
- `reset_adcs!(driver)`: Reset the ADCS flight computer to initial state

# Example Usage in Physics Simulator:
```julia
driver = ADCSDriver("/dev/tty.usbmodem1101")
wait_for_ready(driver)

# In simulation loop:
sensor_data = SensorPacketData(...)
send_sensor_packet!(driver, sensor_data)
actuator_data = receive_actuator_packet(driver)
# Apply actuator_data.magdrv_requested to physics model
```

Usage: julia simInterface.jl <serial_port>
Example: julia simInterface.jl /dev/tty.usbmodem1101
"""

using LibSerialPort

# ============================================================================
# PACKET DEFINITIONS
# ============================================================================

const SENSOR_PACKET_SIZE = 108
const ACTUATOR_PACKET_SIZE = 76

"""
Sensor data sent from simulator to ADCS flight software.
Contains all sensor readings needed for GNC algorithms.
"""
struct SensorPacketData
    sim_mjd::Float32                    # Modified Julian Date
    w_body_raw::NTuple{3, Float32}      # Angular velocity [rad/s] in body frame
    b_field_local::NTuple{3, Float32}   # Magnetic field [T] in body frame
    sun_sensors::NTuple{16, UInt16}     # 16 sun sensor ADC readings
    gps_lat::Float32                    # GPS latitude [deg]
    gps_lon::Float32                    # GPS longitude [deg]
    gps_alt::Float32                    # GPS altitude [m]
    gps_time::Float32                   # GPS time [s]
    UTC_date::NTuple{3, Float32}        # (year, month, day)
    adcs_power::Float32                 # Power consumption [W]
    adcs_voltage::Float32               # Bus voltage [V]
    adcs_current::Float32               # Bus current [A]
end

"""
Actuator commands received from ADCS flight software.
Contains GNC state estimates and control outputs.
"""
struct ActuatorPacketData
    q_eci_to_principal::NTuple{4, Float32}    # Attitude quaternion
    w_principal::NTuple{3, Float32}           # Angular velocity estimate [rad/s]
    sun_vector_principal::NTuple{3, Float32}  # Sun vector in body frame
    magdrv_requested::NTuple{3, Float32}      # Magnetorquer dipole [A·m²]
    w_reaction_wheels::NTuple{4, Float32}     # Reaction wheel speeds [rad/s]
end

# ============================================================================
# DRIVER OBJECT
# ============================================================================

"""
ADCS serial driver maintaining connection state and internal buffers.
"""
mutable struct ADCSDriver
    port::LibSerialPort.SerialPort
    port_name::String
    tx_buffer::Vector{UInt8}      # Pre-allocated transmission buffer
    rx_buffer::Vector{UInt8}      # Accumulation buffer for receiving
    header_search_pos::Int        # Current search position in rx_buffer

    function ADCSDriver(port_name::String, baudrate::Int=115200; wait_for_port::Bool=true)
        # Wait for port to become available (device may be rebooting)
        if wait_for_port
            println("⏳ Waiting for serial port to become available...")
            max_attempts = 100
            port_ready = false
            for attempt in 1:max_attempts
                try
                    # Try to open and immediately close to test availability
                    test_port = LibSerialPort.open(port_name, baudrate)
                    close(test_port)
                    port_ready = true
                    println("✅ Serial port available!")
                    break
                catch e
                    if attempt == max_attempts
                        error("❌ Serial port not available after $(max_attempts) attempts: $e")
                    end
                    sleep(0.1)
                end
            end

            # Wait additional time for device to stabilize
            if port_ready
                println("⏱️  Waiting 2 seconds for device to stabilize...")
                sleep(2.0)
            end
        end

        # Now open the port for real
        port = LibSerialPort.open(port_name, baudrate)
        tx_buffer = Vector{UInt8}(undef, max(SENSOR_PACKET_SIZE, ACTUATOR_PACKET_SIZE))
        rx_buffer = Vector{UInt8}()
        sizehint!(rx_buffer, ACTUATOR_PACKET_SIZE * 2)

        new(port, port_name, tx_buffer, rx_buffer, 1)
    end
end

Base.close(driver::ADCSDriver) = close(driver.port)

# ============================================================================
# LOW-LEVEL SERIALIZATION/DESERIALIZATION
# ============================================================================

"""Serialize sensor packet into buffer. Returns checksum."""
@inline function _serialize_sensor_packet!(buffer::Vector{UInt8}, pkt::SensorPacketData)
    idx = 1

    # Header + padding
    @inbounds buffer[idx] = UInt8('S'); idx += 1
    @inbounds buffer[idx] = UInt8('S'); idx += 1
    @inbounds buffer[idx] = 0x00; idx += 1
    @inbounds buffer[idx] = 0x00; idx += 1

    # Write Float32 values directly to buffer
    @inbounds unsafe_store!(Ptr{Float32}(pointer(buffer, idx)), pkt.sim_mjd); idx += 4

    @inbounds for v in pkt.w_body_raw
        unsafe_store!(Ptr{Float32}(pointer(buffer, idx)), v)
        idx += 4
    end

    @inbounds for v in pkt.b_field_local
        unsafe_store!(Ptr{Float32}(pointer(buffer, idx)), v)
        idx += 4
    end

    @inbounds for v in pkt.sun_sensors
        unsafe_store!(Ptr{UInt16}(pointer(buffer, idx)), v)
        idx += 2
    end

    @inbounds unsafe_store!(Ptr{Float32}(pointer(buffer, idx)), pkt.gps_lat); idx += 4
    @inbounds unsafe_store!(Ptr{Float32}(pointer(buffer, idx)), pkt.gps_lon); idx += 4
    @inbounds unsafe_store!(Ptr{Float32}(pointer(buffer, idx)), pkt.gps_alt); idx += 4
    @inbounds unsafe_store!(Ptr{Float32}(pointer(buffer, idx)), pkt.gps_time); idx += 4

    @inbounds for v in pkt.UTC_date
        unsafe_store!(Ptr{Float32}(pointer(buffer, idx)), v)
        idx += 4
    end

    @inbounds unsafe_store!(Ptr{Float32}(pointer(buffer, idx)), pkt.adcs_power); idx += 4
    @inbounds unsafe_store!(Ptr{Float32}(pointer(buffer, idx)), pkt.adcs_voltage); idx += 4
    @inbounds unsafe_store!(Ptr{Float32}(pointer(buffer, idx)), pkt.adcs_current); idx += 4

    # Calculate checksum (idx-1 is last data byte)
    checksum = UInt16(0)
    @inbounds @simd for i in 1:(idx-1)
        checksum += UInt16(buffer[i])
    end

    # Write checksum and padding
    @inbounds buffer[idx] = UInt8(checksum & 0xFF); idx += 1
    @inbounds buffer[idx] = UInt8((checksum >> 8) & 0xFF); idx += 1
    @inbounds buffer[idx] = 0x00; idx += 1
    @inbounds buffer[idx] = 0x00

    return checksum
end

"""Parse and validate actuator packet. Returns ActuatorPacketData or nothing."""
@inline function _parse_actuator_packet(data::Vector{UInt8})
    @inbounds length(data) != 76 && (return nothing)
    @inbounds (data[1] != UInt8('A') || data[2] != UInt8('A')) && (return nothing)

    # Fast checksum validation
    expected = UInt16(0)
    @inbounds @simd for i in 1:72
        expected += UInt16(data[i])
    end
    @inbounds received = UInt16(data[73]) | (UInt16(data[74]) << 8)
    expected != received && (return nothing)

    # Direct memory reads for much faster parsing
    idx = 5
    @inbounds q = ntuple(i -> (v = unsafe_load(Ptr{Float32}(pointer(data, idx))); idx += 4; v), 4)
    @inbounds w = ntuple(i -> (v = unsafe_load(Ptr{Float32}(pointer(data, idx))); idx += 4; v), 3)
    @inbounds sun = ntuple(i -> (v = unsafe_load(Ptr{Float32}(pointer(data, idx))); idx += 4; v), 3)
    @inbounds mag = ntuple(i -> (v = unsafe_load(Ptr{Float32}(pointer(data, idx))); idx += 4; v), 3)
    @inbounds wheels = ntuple(i -> (v = unsafe_load(Ptr{Float32}(pointer(data, idx))); idx += 4; v), 4)

    return ActuatorPacketData(q, w, sun, mag, wheels)
end

# ============================================================================
# PUBLIC DRIVER API
# ============================================================================

"""
Wait for ADCS to complete initialization and send ready marker.
Blocks until "ADCS_READY" is received or timeout (default 10s).
"""
function wait_for_ready(driver::ADCSDriver; timeout_s::Float64=10.0)
    text_buffer = UInt8[]
    sizehint!(text_buffer, 256)
    ready_marker = b"ADCS_READY"
    start_time = time()

    while (time() - start_time) < timeout_s
        try
            byte = read(driver.port, 1)[1]
            if byte >= 0x20 && byte <= 0x7E || byte == UInt8('\n')  # Printable or newline
                push!(text_buffer, byte)

                # Check for ready marker
                if length(text_buffer) >= length(ready_marker)
                    @inbounds for i in (length(text_buffer)-length(ready_marker)+1):length(text_buffer)
                        if text_buffer[i:i+length(ready_marker)-1] == ready_marker
                            return true
                        end
                    end
                end

                # Prevent unbounded growth
                if length(text_buffer) > 200
                    text_buffer = text_buffer[end-100:end]
                end
            end
        catch e
            sleep(0.001)
        end
    end

    return false  # Timeout
end

"""
Reset the ADCS flight computer by sending a packet of all zeros.
This triggers the flight computer to return to its initial state.
After sending the reset packet, waits for the ADCS to reboot and send ADCS_READY.
"""
function reset_adcs!(driver::ADCSDriver; timeout_s::Float64=10.0)
    println("🔄 Resetting ADCS flight computer...")

    # Clear driver's receive buffer to avoid stale data
    empty!(driver.rx_buffer)
    driver.header_search_pos = 1

    # Send a packet of all zeros WITH HEADER 'SS' (SENSOR_PACKET_SIZE bytes)
    fill!(driver.tx_buffer, 0x00)
    @inbounds driver.tx_buffer[1] = UInt8('S')
    @inbounds driver.tx_buffer[2] = UInt8('S')
    packet_view = view(driver.tx_buffer, 1:SENSOR_PACKET_SIZE)
    write(driver.port, packet_view)

    println("   Reset packet sent, waiting for reboot...")

    # Wait for ADCS to reboot and send ready signal
    if wait_for_ready(driver, timeout_s=timeout_s)
        println("✅ ADCS reset complete and ready!\n")
        return true
    else
        println("⚠️  Warning: ADCS_READY not received after reset\n")
        return false
    end
end

"""
Send sensor packet to ADCS flight software.
This is non-blocking - data is written to serial and returns immediately.
"""
function send_sensor_packet!(driver::ADCSDriver, sensor_data::SensorPacketData)
    # Serialize into driver's tx buffer
    _serialize_sensor_packet!(driver.tx_buffer, sensor_data)

    # Write packet to serial
    packet_view = view(driver.tx_buffer, 1:SENSOR_PACKET_SIZE)
    write(driver.port, packet_view)

    return nothing
end

"""
Receive actuator packet from ADCS flight software.
Blocks until a valid packet is received. Returns ActuatorPacketData.

This function maintains internal state in driver.rx_buffer for streaming reception.
"""
function receive_actuator_packet(driver::ADCSDriver)
    while true
        # Read available data
        bytes_avail = bytesavailable(driver.port)
        if bytes_avail > 0
            chunk_size = min(bytes_avail, 256)
            chunk = read(driver.port, chunk_size)
            append!(driver.rx_buffer, chunk)

            # Search for 'AA' header
            @inbounds for i in driver.header_search_pos:(length(driver.rx_buffer)-1)
                if driver.rx_buffer[i] == UInt8('A') && driver.rx_buffer[i+1] == UInt8('A')
                    # Found potential packet header
                    packet_end = i + ACTUATOR_PACKET_SIZE - 1
                    if packet_end <= length(driver.rx_buffer)
                        # Have full packet, try to parse
                        packet_view = view(driver.rx_buffer, i:packet_end)
                        actuator_pkt = _parse_actuator_packet(collect(packet_view))

                        if !isnothing(actuator_pkt)
                            # Valid packet! Remove processed data and return
                            deleteat!(driver.rx_buffer, 1:packet_end)
                            driver.header_search_pos = 1
                            return actuator_pkt
                        end
                    end
                end
            end

            # Update search position
            driver.header_search_pos = max(1, length(driver.rx_buffer) - 1)

            # Prevent buffer overflow
            if length(driver.rx_buffer) > 1024
                deleteat!(driver.rx_buffer, 1:(length(driver.rx_buffer)-512))
                driver.header_search_pos = 1
            end
        else
            sleep(0.0001)  # No data available, short sleep
        end
    end
end

# ============================================================================
# EXAMPLE/TEST CODE (for standalone testing)
# ============================================================================

"""
Example simulation loop showing how to use the driver in a physics simulator.
This function demonstrates the basic pattern:
  1. Create driver and wait for ADCS ready
  2. In loop: send sensors -> receive actuators -> update physics

In a real physics simulator, you would:
  - Generate sensor_data from your physics state
  - Apply actuator_data.magdrv_requested to your magnetic torque model
  - Apply actuator_data.w_reaction_wheels to your RW momentum model
"""
function example_simulation_loop(port_name::String; update_rate_hz::Int=1000)
    println("🚀 ADCS Simulation Interface - Example Loop\n")

    # Initialize driver
    println("📡 Connecting to $port_name...")
    driver = ADCSDriver(port_name)

    # Reset ADCS to ensure clean state before simulation
    reset_adcs!(driver)

    # Initial simulation time
    sim_mjd_start = 59000.5f0  # Starting Modified Julian Date
    dt_seconds = 0.1f0
    dt_days = dt_seconds / 86400.0f0  # Time step in days
    gps_time_start = 120000.0f0

    try
        packet_count = 0
        last_time = time()

        while true
            packet_count += 1

            # Update magnetic field (normalized, varies sinusoidally)
            t = Float32(packet_count) / Float32(update_rate_hz)  # Time in seconds
            theta = 2.0f0 * Float32(π) * t / 60.0f0  # One rotation per minute
            b_mag = (sin(theta), cos(theta), sin(2.0f0 * theta))
            b_norm = sqrt(b_mag[1]^2 + b_mag[2]^2 + b_mag[3]^2)
            b_field_normalized = (b_mag[1]/b_norm, b_mag[2]/b_norm, b_mag[3]/b_norm)

            # Increment MJD and GPS time
            current_mjd = sim_mjd_start + Float32(packet_count) * dt_days
            current_gps_time = gps_time_start + Float32(packet_count) * dt_seconds

            # Convert MJD to calendar date for UTC_date field
            # Simplified: just increment from start date
            days_elapsed = Float32(floor(Float32(packet_count) * dt_days))
            year = 2025.0f0
            month = 10.0f0
            day = 13.0f0 + days_elapsed

            # Create sensor data with updated values
            sensor_data = SensorPacketData(
                current_mjd,                        # mjd (incrementing)
                (0.1f0, 0.2f0, 0.3f0),             # angular velocity
                b_field_normalized,                 # magnetic field (normalized, sinusoidal)
                ntuple(i -> UInt16(1000 + i), 16), # sun sensors
                37.4f0, -122.1f0, 0.0f0,           # gps lat/lon/alt
                current_gps_time,                   # gps time (incrementing)
                (year, month, day),                 # UTC date (incrementing)
                5.0f0, 12.0f0, 0.417f0             # power
            )

            # STEP 1: Send sensor data to ADCS
            send_sensor_packet!(driver, sensor_data)
            println("📤 Sent sensor packet #$packet_count")
            # println("   sim_mjd: $(sensor_data.sim_mjd)")
            # println("   w_body_raw: $(sensor_data.w_body_raw)")
            println("   b_field_local: $(sensor_data.b_field_local)")
            # println("   sun_sensors: $(sensor_data.sun_sensors)")
            # println("   gps_lat: $(sensor_data.gps_lat), gps_lon: $(sensor_data.gps_lon), gps_alt: $(sensor_data.gps_alt)")
            println("   gps_time: $(sensor_data.gps_time)")
            println("   UTC_date: $(sensor_data.UTC_date)")
            # println("   adcs_power: $(sensor_data.adcs_power), adcs_voltage: $(sensor_data.adcs_voltage), adcs_current: $(sensor_data.adcs_current)")

            # First packet primes the system, skip receive
            if packet_count == 1
                sleep(1.0 / update_rate_hz)
                last_time = time()
                continue
            end

            # STEP 2: Receive actuator commands from ADCS
            actuator_data = receive_actuator_packet(driver)
            println("📥 Received actuator packet #$(packet_count-1)")
            println("   Magnetorquer: $(actuator_data.magdrv_requested)")
            println("   Angular vel:  $(actuator_data.w_principal)\n")

            # STEP 3: In real simulator, apply actuator_data to physics model here
            # e.g., torque = cross(actuator_data.magdrv_requested, b_field_eci)

            # Rate limiting
            current_time = time()
            elapsed = current_time - last_time
            if elapsed < 1.0 / update_rate_hz
                sleep(1.0 / update_rate_hz - elapsed)
            end
            last_time = time()
        end
    catch e
        if isa(e, InterruptException)
            println("\n✅ Simulation stopped. Sent $packet_count packets successfully")
        else
            rethrow(e)
        end
    finally
        close(driver)
    end
end

# ============================================================================
# MAIN (for standalone execution)
# ============================================================================

if abspath(PROGRAM_FILE) == @__FILE__
    if length(ARGS) < 1
        println("Usage: julia simInterface.jl <serial_port>")
        println("Example: julia simInterface.jl /dev/tty.usbmodem1101")
        exit(1)
    end

    example_simulation_loop(ARGS[1])
end
