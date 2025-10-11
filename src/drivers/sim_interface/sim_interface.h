/**
 * @file sim_interface.h
 * @brief Simulation interface for hardware-in-the-loop testing
 *
 * Provides USB serial communication between external physics simulator
 * and the flight computer. In SIMULATION mode, replaces real sensor
 * reads with data from the simulator and sends actuator commands back.
 */

#pragma once

#include "slate.h"
#include <stdint.h>

#ifdef SIMULATION

/**
 * @brief Packet structure for sensor data from simulator to flight computer
 *
 * All floats are little-endian IEEE 754
 */
typedef struct __attribute__((packed)) {
    // Header
    uint8_t header[2];  // "SS" (0x53 0x53)

    // IMU data
    float w_body_x;     // [rad/s]
    float w_body_y;     // [rad/s]
    float w_body_z;     // [rad/s]

    // Magnetometer data
    float b_field_x;    // [unit vector component]
    float b_field_y;    // [unit vector component]
    float b_field_z;    // [unit vector component]

    // GPS data
    float gps_lat;      // [degrees] N+ S-
    float gps_lon;      // [degrees] E+ W-
    float gps_time;     // [s]

    // Sun sensor data (16 sensors - all of them)
    uint16_t sun_sensors[16];  // [0-4095] ADC values

    // Footer
    uint16_t checksum;  // Simple sum of all bytes
} sim_sensor_packet_t;

/**
 * @brief Packet structure for actuator commands from flight computer to simulator
 *
 * All floats are little-endian IEEE 754
 */
typedef struct __attribute__((packed)) {
    // Header
    uint8_t header[2];  // "AA" (0x41 0x41)

    // Magnetorquer requests
    float magdrv_x;     // [-1.0 to 1.0]
    float magdrv_y;     // [-1.0 to 1.0]
    float magdrv_z;     // [-1.0 to 1.0]

    // Reaction wheel requests (3 wheels)
    float rw_speeds[3]; // [rad/s]

    // Footer
    uint16_t checksum;  // Simple sum of all bytes
} sim_actuator_packet_t;

/**
 * @brief Initialize simulation interface
 *
 * Sets up USB serial communication and internal buffers
 */
void sim_init();

/**
 * @brief Read sensor data from simulator and populate slate
 *
 * Blocking with timeout. Waits for a complete packet from simulator.
 *
 * @param slate Pointer to slate structure to populate
 * @param timeout_ms Maximum time to wait for packet in milliseconds (0 = non-blocking)
 * @return true if new sensor data received, false if timeout
 */
bool sim_read_sensors(slate_t *slate, uint32_t timeout_ms);

/**
 * @brief Send actuator commands to simulator
 *
 * Reads actuator requests from slate and sends to simulator via USB
 *
 * @param slate Pointer to slate structure containing actuator requests
 */
void sim_send_actuators(slate_t *slate);

/**
 * @brief Get simulation time acceleration factor
 *
 * @return Time multiplier (e.g., 10.0 means 10x real-time)
 */
float sim_get_time_multiplier();

/**
 * @brief Set simulation time acceleration factor
 *
 * @param multiplier Time acceleration (e.g., 10.0 for 10x speed)
 */
void sim_set_time_multiplier(float multiplier);

#endif // SIMULATION
