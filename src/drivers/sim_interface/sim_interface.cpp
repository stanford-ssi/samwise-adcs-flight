/**
 * @author Lundeen Cahilly
 * @date 2025-10-13
 *
 * Implementation of simulation interface for hardware-in-the-loop testing
 *
 * Packet structure is defined in sim_interface.h with __attribute__((packed))
 * to ensure no padding. All multi-byte values are little-endian.
 *
 * Checksum algorithm: Simple sum of all bytes in packet (excluding checksum
 * field itself). Cast packet to uint8_t* and sum from start to end-2 bytes.
 */

#include "sim_interface.h"

#ifdef SIMULATION

#include "constants.h"
#include "gnc/utils/utils.h"
#include "macros.h"
#include "pico/stdlib.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// Static state
static sim_sensor_packet_t sensor_packet;
static sim_actuator_packet_t actuator_packet;
static slate_t *global_slate_ptr =
    nullptr; // For time functions to access slate

// Calculate simple checksum
static uint16_t calculate_checksum(const uint8_t *data, size_t len)
{
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;
}

void sim_init()
{
    memset(&sensor_packet, 0, sizeof(sensor_packet));
    memset(&actuator_packet, 0, sizeof(actuator_packet));
    global_slate_ptr = nullptr;
}

bool sim_read_sensors(slate_t *slate, uint32_t timeout_ms)
{
    static uint8_t *packet_ptr = (uint8_t *)&sensor_packet;
    static size_t bytes_received = 0;
    static bool synced = false;
    static int packet_counter = 0;
    static int call_counter = 0;
    call_counter++;

    // Calculate timeout deadline using system time (not simulated time)
    // This is critical because slate->MJD doesn't change until we receive a
    // packet, so using sim_get_absolute_time() would make the timeout check
    // useless
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);

    while (true)
    {
        // Check if we've exceeded timeout using real system time
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0)
        {
            LOG_DEBUG("[sim] Timeout waiting for sensor packet");
            return false;
        }

        // Read byte with short timeout
        int byte = getchar_timeout_us(1000);
        if (byte == PICO_ERROR_TIMEOUT)
        {
            continue;
        }

        // Sync on header
        if (!synced)
        {
            if (bytes_received == 0 && byte == 'S')
            {
                packet_ptr[0] = byte;
                bytes_received = 1;
                LOG_DEBUG("[sim] Got first 'S' for packet sync");
            }
            else if (bytes_received == 1 && byte == 'S')
            {
                packet_ptr[1] = byte;
                bytes_received = 2;
                synced = true;
                packet_counter++;
                LOG_DEBUG("[sim] Synced on 'SS' header, starting packet #%d "
                          "reception",
                          packet_counter);
            }
            else
            {
                if (bytes_received > 0)
                {
                    LOG_DEBUG("[sim] Lost sync, got byte 0x%02X instead of 'S'",
                              byte);
                }
                bytes_received = 0;
            }
            continue;
        }

        // Accumulate packet (with bounds checking)
        if (bytes_received < sizeof(sim_sensor_packet_t))
        {
            packet_ptr[bytes_received++] = byte;
        }
        else
        {
            // Buffer overflow protection - should never happen but reset if it
            // does
            LOG_ERROR(
                "[sim] Buffer overflow detected! bytes_received=%zu, max=%zu",
                bytes_received, sizeof(sim_sensor_packet_t));
            bytes_received = 0;
            synced = false;
            continue;
        }

        // Check if complete
        if (bytes_received >= sizeof(sim_sensor_packet_t))
        {
            // Check for reset packet (all zeros) - do this BEFORE checksum
            // validation An all-zero packet will have checksum=0 (since sum of
            // zeros is zero) This is the reset signal from the simulator to
            // return to init state
            bool is_reset_packet = true;
            const uint8_t *packet_bytes = (const uint8_t *)&sensor_packet;
            for (size_t i = 0; i < sizeof(sim_sensor_packet_t); i++)
            {
                if (packet_bytes[i] != 0)
                {
                    is_reset_packet = false;
                    break;
                }
            }

            if (is_reset_packet)
            {
                LOG_INFO("[sim] *** RESET PACKET RECEIVED - Returning to init "
                         "state ***");
                bytes_received = 0;
                synced = false;

                // Reset simulation time
                slate->sim_time_initialized = false;

                // Transition to init state
                extern sched_state_t init_state;
                slate->current_state = &init_state;
                slate->entered_current_state_time = get_absolute_time();

                // Signal that ADCS is ready after reset
                printf("ADCS_READY\n");
                fflush(stdout);

                return false; // Return false to indicate reset (no sensor data)
            }

            // Verify checksum - calculate over bytes up to (but not including)
            // checksum field This is: header(2) + padding(2) + all data
            // fields(100) = 104 bytes
            size_t checksum_offset = offsetof(sim_sensor_packet_t, checksum);
            uint16_t calc_crc =
                calculate_checksum((uint8_t *)&sensor_packet, checksum_offset);

            if (calc_crc != sensor_packet.checksum)
            {
                LOG_ERROR("[sim] Checksum mismatch! Calculated: 0x%04X, "
                          "Received: 0x%04X - discarding packet",
                          calc_crc, sensor_packet.checksum);
                bytes_received = 0;
                synced = false;
                continue;
            }

            // Initialize simulation time on first packet
            if (!slate->sim_time_initialized)
            {
                slate->sim_t0_mjd = sensor_packet.sim_mjd;
                slate->sim_t0_system = get_absolute_time();
                slate->sim_time_initialized = true;
                // LOG_INFO("[sim] Time initialized: t0_mjd=%.6f",
                // slate->sim_t0_mjd);
            }

            // Update current MJD from simulator
            slate->MJD = sensor_packet.sim_mjd;

            // GPS
            slate->gps_alive = true;
            slate->gps_lat = sensor_packet.gps_lat;
            slate->gps_lon = sensor_packet.gps_lon;
            slate->gps_alt = sensor_packet.gps_alt;
            slate->gps_time = sensor_packet.gps_time;
            slate->gps_data_valid = true;

            // IMU
            slate->imu_alive = true;
            slate->w_body_raw = sensor_packet.w_body_raw;
            constexpr float imu_lpf_alpha = 0.628318530718;
            slate->w_body_filtered = low_pass_filter(
                slate->w_body_filtered, slate->w_body_raw, imu_lpf_alpha);
            slate->w_mag = length(slate->w_body_filtered);
            slate->imu_data_valid = true;

            // Magnetometer
            slate->magmeter_alive = true;
            slate->b_field_local = sensor_packet.b_field_local;
            slate->magmeter_data_valid = true;
            slate->b_field_read_time = sim_get_absolute_time(slate);
            slate->bdot_data_has_updated = true; // Set flag for bdot

            // Sun sensors
            for (int i = 0; i < NUM_SUN_SENSORS; i++)
            {
                slate->sun_sensors_intensities[i] =
                    sensor_packet.sun_sensors[i];
                slate->sun_sensors_voltages[i] =
                    (sensor_packet.sun_sensors[i] / 4095.0f) * 3.3f;
            }
            slate->sun_pyramids_alive = true;
            slate->photodiodes_yz_alive = true;
            slate->sun_pyramids_data_valid = true;
            slate->photodiodes_yz_data_valid = true;

            // Reset for next packet
            bytes_received = 0;
            synced = false;
            return true;
        }
    }
}

void sim_send_actuators(slate_t *slate)
{
    // Populate header
    actuator_packet.header[0] = 'A';
    actuator_packet.header[1] = 'A';

    // Attitude state
    actuator_packet.q_eci_to_principal = slate->q_eci_to_principal;
    actuator_packet.w_principal = slate->w_principal;
    actuator_packet.sun_vector_principal = slate->sun_vector_principal;

    // Actuator requests
    actuator_packet.magdrv_requested = slate->magdrv_requested;
    for (int i = 0; i < 4 && i < NUM_REACTION_WHEELS; i++)
    {
        actuator_packet.w_reaction_wheels[i] =
            slate->reaction_wheels_w_requested[i];
    }

    // Calculate checksum over bytes up to (but not including) checksum field
    size_t checksum_offset = offsetof(sim_actuator_packet_t, checksum);
    actuator_packet.checksum =
        calculate_checksum((uint8_t *)&actuator_packet, checksum_offset);

    // Send packet (binary data)
    size_t written =
        fwrite(&actuator_packet, sizeof(sim_actuator_packet_t), 1, stdout);
    fflush(stdout);
}

absolute_time_t sim_get_absolute_time(slate_t *slate)
{
    // Use GPS time directly (GPS time is in seconds, convert to microseconds)
    // This gives us the absolute simulation time based on GPS clock
    // TODO: Make this pull from an absolute time set by sim w/o noise
    return (absolute_time_t)(slate->gps_time * 1000000.0f);
}

absolute_time_t sim_make_timeout_time_ms(slate_t *slate, uint32_t ms)
{
    absolute_time_t now = sim_get_absolute_time(slate);
    return now + ((int64_t)ms * 1000);
}

#endif // SIMULATION
