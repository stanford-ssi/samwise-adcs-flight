#!/usr/bin/env julia
"""
Minimal test to send sensor packets and receive actuator packets

Usage: julia test_sim_interface.jl <serial_port>
Example: julia test_sim_interface.jl /dev/tty.usbmodem1101
"""

using LibSerialPort

# Pre-allocate buffers for better performance
const UPDATE_RATE = 1000  # Hz
const SENSOR_PACKET_SIZE = 108
const ACTUATOR_PACKET_SIZE = 76
const PACKET_BUFFER = Vector{UInt8}(undef, max(SENSOR_PACKET_SIZE, ACTUATOR_PACKET_SIZE))
const READ_BUFFER = Vector{UInt8}(undef, 1024)  # Larger buffer for bulk reads

# Packet structures
struct SensorPacketData
    sim_mjd::Float32
    w_body_raw::NTuple{3, Float32}
    b_field_local::NTuple{3, Float32}
    sun_sensors::NTuple{16, UInt16}
    gps_lat::Float32
    gps_lon::Float32
    gps_alt::Float32
    gps_time::Float32
    UTC_date::NTuple{3, Float32}
    adcs_power::Float32
    adcs_voltage::Float32
    adcs_current::Float32
end

struct ActuatorPacketData
    q_eci_to_principal::NTuple{4, Float32}
    w_principal::NTuple{3, Float32}
    sun_vector_principal::NTuple{3, Float32}
    magdrv_requested::NTuple{3, Float32}
    w_reaction_wheels::NTuple{4, Float32}
end

@inline function serialize_packet!(buffer::Vector{UInt8}, pkt::SensorPacketData)
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

@inline function parse_actuator_packet(data::Vector{UInt8})
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

function test_packets(port_name::String)
    println("🚀 Testing packet send/receive\n")
    println("Reference bytes (header SS=green, AA=yellow):")

    # Create test packet
    pkt = SensorPacketData(
        59000.5f0,                          # mjd
        (0.1f0, 0.2f0, 0.3f0),             # angular velocity
        (0.707f0, 0.0f0, 0.707f0),         # magnetic field
        ntuple(i -> UInt16(1000 + i), 16), # sun sensors
        37.4f0, -122.1f0, 0.0f0, 120000.0f0, # gps
        (2025.0f0, 10.0f0, 13.0f0),        # date
        5.0f0, 12.0f0, 0.417f0             # power
    )

    # Pre-serialize the packet into the global buffer
    full_packet = view(PACKET_BUFFER, 1:SENSOR_PACKET_SIZE)
    checksum = serialize_packet!(PACKET_BUFFER, pkt)
    println("\n📦 Sensor packet ($SENSOR_PACKET_SIZE bytes, checksum: 0x$(string(checksum, base=16, pad=4)))")

    # Wait for port to become available (device may be rebooting)
    println("\n⏳ Waiting for serial port to become available...")
    max_attempts = 100000000
    for attempt in 1:max_attempts
        try
            sp = LibSerialPort.SerialPort(port_name)
            close(sp)
            println("✅ Serial port ready!\n")
            break
        catch e
            if attempt == max_attempts
                println("❌ Serial port not available after $(max_attempts) attempts")
                rethrow(e)
            end
            print(".")
            sleep(0.01)
        end
    end

    LibSerialPort.open(port_name, 115200) do sp
        println("\n⏳ Waiting for ADCS_READY marker...")

        # Wait for "ADCS_READY" startup marker from flight code
        ready = false
        text_buffer = UInt8[]
        max_wait_s = 10.0
        start_time = time()
        ready_marker = b"ADCS_READY"

        sizehint!(text_buffer, 256)  # Pre-allocate

        while !ready && (time() - start_time) < max_wait_s
            try
                byte = read(sp, 1)[1]
                if byte >= 0x20 && byte <= 0x7E || byte == UInt8('\n')  # Printable or newline
                    push!(text_buffer, byte)

                    # Check if we got the ready marker (faster byte comparison)
                    if length(text_buffer) >= length(ready_marker)
                        found = false
                        @inbounds for i in (length(text_buffer)-length(ready_marker)+1):length(text_buffer)
                            if text_buffer[i:i+length(ready_marker)-1] == ready_marker
                                println("✅ ADCS is ready!\n")
                                ready = true
                                found = true
                                break
                            end
                        end
                    end

                    # Clear buffer if it gets too long
                    if length(text_buffer) > 200
                        text_buffer = text_buffer[end-100:end]
                    end
                end
            catch e
                # Handle timeout or other errors
                sleep(0.001)  # Much shorter sleep for faster response
            end
        end

        if !ready
            println("⚠️  Warning: ADCS_READY marker not received within $(max_wait_s)s, continuing anyway...\n")
        end

        try
            packet_count = 0
            buffer = Vector{UInt8}()
            sizehint!(buffer, ACTUATOR_PACKET_SIZE * 2)  # Pre-allocate with extra space

            header_search_start = 1
            last_time = time()

            while true
                packet_count += 1

                # Send sensor packet (use pre-serialized buffer)
                write(sp, full_packet)
                # No flush needed - LibSerialPort handles it
                println("📤 Sent sensor packet #$packet_count")

                packet_count == 1 && continue  # First packet primes the system

                # Clear buffer for new actuator packet
                empty!(buffer)
                header_search_start = 1

                # Wait for actuator response
                println("   ⏳ Waiting for actuator packet...")
                found_packet = false

                while !found_packet
                    # Try bulk read for better performance
                    bytes_avail = bytesavailable(sp)
                    if bytes_avail > 0
                        # Read available bytes
                        chunk_size = min(bytes_avail, 256)
                        chunk = read(sp, chunk_size)
                        append!(buffer, chunk)

                        # Search for AA header in newly added data
                        @inbounds for i in header_search_start:(length(buffer)-1)
                            if buffer[i] == UInt8('A') && buffer[i+1] == UInt8('A')
                                # Found potential header - check if we have full packet
                                packet_end = i + ACTUATOR_PACKET_SIZE - 1
                                if packet_end <= length(buffer)
                                    packet_view = view(buffer, i:packet_end)
                                    actuator_pkt = parse_actuator_packet(collect(packet_view))
                                    if !isnothing(actuator_pkt)
                                        println("📥 Actuator packet #$(packet_count-1) received ✅")
                                        found_packet = true
                                        # Remove processed data
                                        deleteat!(buffer, 1:packet_end)
                                        header_search_start = 1
                                        break
                                    end
                                end
                            end
                        end

                        # Update search position for next iteration
                        header_search_start = max(1, length(buffer) - 1)

                        # Prevent buffer from growing unbounded
                        if length(buffer) > 1024
                            deleteat!(buffer, 1:(length(buffer)-512))
                            header_search_start = 1
                        end
                    else
                        # No data available, short sleep
                        sleep(0.0001)
                    end
                end

                # Precise timing control for consistent update rate
                current_time = time()
                elapsed = current_time - last_time
                if elapsed < 1 / UPDATE_RATE
                    sleep(1 / UPDATE_RATE - elapsed)
                end
                last_time = time()
            end
        catch e
            isa(e, InterruptException) && println("\n\n✅ Sent $packet_count packets successfully")
            !isa(e, InterruptException) && rethrow(e)
        end
    end
end

# Main
if length(ARGS) < 1
    println("Usage: julia test_sim_interface.jl <serial_port>")
    println("Example: julia test_sim_interface.jl /dev/tty.usbmodem1101")
    exit(1)
end

test_packets(ARGS[1])
