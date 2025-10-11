/**
 * @author Lundeen Cahilly
 * @date 2025-08-20
 *
 * Implementation of simulation interface for hardware-in-the-loop testing
 */

#include "sim_interface.h"

#ifdef SIMULATION

#include "constants.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

// Time acceleration factor
static float time_multiplier = 1.0f;

// Internal buffers
static sim_sensor_packet_t sensor_packet;
static sim_actuator_packet_t actuator_packet;

// Simple checksum calculation
static uint16_t calculate_checksum(const uint8_t *data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

void sim_init() {
    // USB stdio is already initialized in main
    // Just clear our buffers
    memset(&sensor_packet, 0, sizeof(sensor_packet));
    memset(&actuator_packet, 0, sizeof(actuator_packet));

    // Set default time multiplier
    time_multiplier = 1.0f;

    printf("[SIM] Simulation interface initialized\n");
    printf("[SIM] Waiting for sensor packets (header: 'SS')\n");
    printf("[SIM] Sensor packet size: %d bytes\n", sizeof(sim_sensor_packet_t));
    printf("[SIM] Actuator packet size: %d bytes\n", sizeof(sim_actuator_packet_t));
}

bool sim_read_sensors(slate_t *slate, uint32_t timeout_ms) {
    static uint8_t *packet_ptr = (uint8_t *)&sensor_packet;
    static size_t bytes_received = 0;
    static bool synced = false;

    absolute_time_t start_time = get_absolute_time();
    absolute_time_t deadline = delayed_by_ms(start_time, timeout_ms);

    // Keep trying until we get a complete packet or timeout
    while (true) {
        // Check for timeout
        if (timeout_ms > 0 && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
            printf("[SIM] Timeout waiting for sensor packet\n");
            return false;
        }

        // Try to read a byte (with small timeout to avoid busy-wait)
        int available = getchar_timeout_us(1000);  // 1ms timeout per byte
        if (available == PICO_ERROR_TIMEOUT) {
            continue;  // Keep trying until overall timeout
        }

        uint8_t byte = (uint8_t)available;

        // Look for header sync
        if (!synced) {
            if (bytes_received == 0 && byte == 'S') {
                packet_ptr[0] = byte;
                bytes_received = 1;
            } else if (bytes_received == 1 && byte == 'S') {
                packet_ptr[1] = byte;
                bytes_received = 2;
                synced = true;
            } else {
                bytes_received = 0;
            }
            continue;
        }

        // Accumulate packet bytes
        packet_ptr[bytes_received++] = byte;

        // Check if we have a complete packet
        if (bytes_received >= sizeof(sim_sensor_packet_t)) {
            // Verify checksum
            uint16_t calc_checksum = calculate_checksum(
                (uint8_t *)&sensor_packet,
                sizeof(sim_sensor_packet_t) - sizeof(uint16_t));

            if (calc_checksum == sensor_packet.checksum) {
                // Valid packet - populate slate
                slate->w_body_raw = {sensor_packet.w_body_x,
                                      sensor_packet.w_body_y,
                                      sensor_packet.w_body_z};
                slate->w_body_filtered = slate->w_body_raw;
                slate->w_mag = length(slate->w_body_raw);
                slate->imu_data_valid = true;
                slate->imu_alive = true;

                slate->b_field_local = {sensor_packet.b_field_x,
                                         sensor_packet.b_field_y,
                                         sensor_packet.b_field_z};
                slate->b_field_read_time = get_absolute_time();
                slate->magmeter_data_valid = true;
                slate->magmeter_alive = true;

                slate->gps_lat = sensor_packet.gps_lat;
                slate->gps_lon = sensor_packet.gps_lon;
                slate->gps_time = sensor_packet.gps_time;
                slate->gps_data_valid = true;
                slate->gps_alive = true;

                // Copy all 16 sun sensor data
                for (int i = 0; i < NUM_SUN_SENSORS; i++) {
                    slate->sun_sensors_intensities[i] = sensor_packet.sun_sensors[i];
                    slate->sun_sensors_voltages[i] =
                        (sensor_packet.sun_sensors[i] / 4095.0f) * 3.3f;
                }
                slate->sun_pyramids_data_valid = true;
                slate->sun_pyramids_alive = true;
                slate->photodiodes_yz_data_valid = true;
                slate->photodiodes_yz_alive = true;

                // Reset for next packet
                bytes_received = 0;
                synced = false;

                return true;
            } else {
                // Checksum failed - resync
                printf("[SIM] Checksum failed: expected 0x%04X, got 0x%04X\n",
                       calc_checksum, sensor_packet.checksum);
                bytes_received = 0;
                synced = false;
            }
        }
    }
}

void sim_send_actuators(slate_t *slate) {
    // Populate actuator packet
    actuator_packet.header[0] = 'A';
    actuator_packet.header[1] = 'A';

    actuator_packet.magdrv_x = slate->magdrv_requested.x;
    actuator_packet.magdrv_y = slate->magdrv_requested.y;
    actuator_packet.magdrv_z = slate->magdrv_requested.z;

    // Copy reaction wheel speeds (up to 3)
    for (int i = 0; i < 3 && i < NUM_REACTION_WHEELS; i++) {
        actuator_packet.rw_speeds[i] = slate->reaction_wheels_w_requested[i];
    }

    // Calculate and add checksum
    actuator_packet.checksum = calculate_checksum(
        (uint8_t *)&actuator_packet,
        sizeof(sim_actuator_packet_t) - sizeof(uint16_t));

    // Send packet over USB
    fwrite(&actuator_packet, sizeof(sim_actuator_packet_t), 1, stdout);
    fflush(stdout);
}

float sim_get_time_multiplier() {
    return time_multiplier;
}

void sim_set_time_multiplier(float multiplier) {
    time_multiplier = multiplier;
    printf("[SIM] Time multiplier set to %.2fx\n", multiplier);
}

#endif // SIMULATION
