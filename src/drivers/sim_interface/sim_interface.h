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

// #ifdef SIMULATION

/**
 * @brief Packet structure for sensor data from simulator to flight computer
 *
 * All floats are little-endian IEEE 754
 */
typedef struct __attribute__((packed)) {
    // Header
    uint8_t header[2];  // "SS" (0x53 0x53)
    float sim_mjd;          // MJD according to simulator w/ fractional day

    // IMU data
    float3 w_body_raw;  // [rad/s] in body frame

    // Magnetometer data
    float3 b_field_local;   // B field in body frame (unit vector)

    // Sun sensor data (16 sensors - all of them)
    uint16_t sun_sensors[16];  // [0-3102] ADC values

    // GPS data
    float gps_lat;      // [degrees] N+ S-
    float gps_lon;      // [degrees] E+ W-
    float gps_alt;      // [km]
    float gps_time;     // [HHMMSS as float]
    float3 UTC_date;   // [year, month, day]

    // Power monitoring
    float adcs_power;   // [W] ADCS board power consumption
    float adcs_voltage; // [V] ADCS board voltage
    float adcs_current; // [A] ADCS board current

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

    // Attitude propagator
    quaternion q_eci_to_body;
    float3 w_body;   // [rad s^-1] in body frame
    float3 sun_vector_body; // (unit vector) in body frame

    // Magnetorquer requests
    float3 magdrv_requested;     // [-1.0 to 1.0] in body frame

    // Reaction wheel requests (3 wheels)
    float reaction_wheels_w_requested[3]; // [rad/s]

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
