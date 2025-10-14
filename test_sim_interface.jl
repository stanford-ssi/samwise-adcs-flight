#!/usr/bin/env julia
"""
Minimal test to send sensor packets and receive actuator packets with colorful output.

Usage: julia test_sim_interface.jl <serial_port>
Example: julia test_sim_interface.jl /dev/tty.usbmodem1101
"""

using LibSerialPort, Printf

# Colorful byte printing
function byte_to_rgb(b)
    val = Int(b)
    if val == 0x53         # 'S'
        return 0, 255, 0   # bright green
    elseif val == 0x41     # 'A'
        return 255, 255, 0 # bright yellow
    else
        return 255, 255, 255 # white
    end
end

function print_colored_bytes(bytes)
    for b in bytes
        r, g, bl = byte_to_rgb(b)
        print("\e[48;2;$(r);$(g);$(bl)m")
        @printf("%02X", b)
        print("\e[0m ")
    end
    println()
end

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

function serialize_packet(pkt::SensorPacketData)
    buf = IOBuffer()
    write(buf, UInt8('S'), UInt8('S'), UInt16(0))  # Header + padding
    write(buf, pkt.sim_mjd)
    for v in pkt.w_body_raw; write(buf, v); end
    for v in pkt.b_field_local; write(buf, v); end
    for v in pkt.sun_sensors; write(buf, v); end
    write(buf, pkt.gps_lat, pkt.gps_lon, pkt.gps_alt, pkt.gps_time)
    for v in pkt.UTC_date; write(buf, v); end
    write(buf, pkt.adcs_power, pkt.adcs_voltage, pkt.adcs_current)

    data = take!(buf)
    checksum = UInt16(sum(UInt16(b) for b in data))
    push!(data, UInt8(checksum & 0xFF), UInt8((checksum >> 8) & 0xFF))
    push!(data, UInt8(0), UInt8(0))  # Trailing padding for alignment (108 bytes total)
    return data, checksum
end

function parse_actuator_packet(data::Vector{UInt8})
    length(data) != 76 && (return nothing)
    data[1] != UInt8('A') || data[2] != UInt8('A') && (return nothing)

    expected = UInt16(sum(UInt16(b) for b in data[1:72]))
    received = UInt16(data[73]) | (UInt16(data[74]) << 8)
    expected != received && (return nothing)

    buf = IOBuffer(data[5:end])
    q = ntuple(i -> read(buf, Float32), 4)
    w = ntuple(i -> read(buf, Float32), 3)
    sun = ntuple(i -> read(buf, Float32), 3)
    mag = ntuple(i -> read(buf, Float32), 3)
    wheels = ntuple(i -> read(buf, Float32), 4)
    return ActuatorPacketData(q, w, sun, mag, wheels)
end

function test_packets(port_name::String)
    println("🚀 Testing packet send/receive\n")
    println("Reference bytes (header SS=green, AA=yellow):")
    print_colored_bytes([0x53, 0x53, 0x41, 0x41])

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

    full_packet, checksum = serialize_packet(pkt)
    println("\n📦 Sensor packet ($(length(full_packet)) bytes, checksum: 0x$(string(checksum, base=16, pad=4)))")
    print_colored_bytes(full_packet)

    # Wait for port to become available (device may be rebooting)
    println("\n⏳ Waiting for serial port to become available...")
    max_attempts = 20
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
        println("\n⏳ Waiting for initialization...")
        sleep(5.0)  # Wait for device to initialize

        try
            packet_count = 0
            buffer = UInt8[]
            text_buffer = ""

            while true
                packet_count += 1

                # Send sensor packet
                write(sp, full_packet)
                flush(sp)
                println("📤 Sent sensor packet #$packet_count")

                packet_count == 1 && continue  # First packet primes the system

                # Reset buffer for new actuator packet
                buffer = UInt8[]

                # Wait for actuator response
                println("   ⏳ Waiting for actuator packet...")
                while true
                    byte = read(sp, 1)[1]

                    # Print every byte as hex
                    push!(buffer, byte)
                    # r, g, bl = byte_to_rgb(byte)
                    # print("\e[48;2;$(r);$(g);$(bl)m")
                    # @printf("%02X", byte)
                    # print("\e[0m ")

                    # Look for "AA" header
                    if length(buffer) >= 2 && buffer[end-1] == UInt8('A') && buffer[end] == UInt8('A')
                        buffer = UInt8[UInt8('A'), UInt8('A')]
                    end

                    # Complete packet?
                    if length(buffer) == 76 && buffer[1] == UInt8('A') && buffer[2] == UInt8('A')
                        actuator_pkt = parse_actuator_packet(buffer)
                        if !isnothing(actuator_pkt)
                            println("\n📥 Actuator packet #$packet_count received ✅")
                            # println("   q: $(actuator_pkt.q_eci_to_principal)")
                            # println("   ω: $(actuator_pkt.w_principal) rad/s")
                            # println("   mag: $(actuator_pkt.magdrv_requested) A·m²\n")
                            break
                        else
                            println("\n   ❌ Checksum failed!")
                        end
                        buffer = UInt8[]
                    end
                end
                sleep(0.01)  # 100Hz update rate
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
