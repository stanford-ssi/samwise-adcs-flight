/**
 * @author Lundeen Cahilly
 * @date 2025-10-13
 *
 * USB serial communication interface for external physics simulator.
 * Replaces real sensor reads with simulator data and sends actuator commands.
 *
 * PROTOCOL:
 *   Sensor packets: Simulator -> Flight Computer
 *     Header: "SS" (0x53 0x53) - 2 bytes
 *     Body: See sim_sensor_packet_t below
 *     Checksum: uint16_t sum of all bytes except checksum field
 *     Total size: 108 bytes (includes 2 bytes padding after header)
 *
 *     Memory layout (all little-endian):
 *       [0-1]    uint8_t[2]   header ("SS")
 *       [2-3]    <padding>
 *       [4-7]    float        sim_mjd
 *       [8-19]   float3       w_body_raw (x, y, z)
 *       [20-31]  float3       b_field_local (x, y, z)
 *       [32-63]  uint16_t[16] sun_sensors
 *       [64-67]  float        gps_lat
 *       [68-71]  float        gps_lon
 *       [72-75]  float        gps_alt
 *       [76-79]  float        gps_time
 *       [80-91]  float3       UTC_date (year, month, day)
 *       [92-95]  float        adcs_power
 *       [96-99]  float        adcs_voltage
 *       [100-103] float       adcs_current
 *       [104-105] uint16_t    checksum
 *       [106-107] <padding>
 *
 *   Actuator packets: Flight Computer -> Simulator
 *     Header: "AA" (0x41 0x41) - 2 bytes
 *     Body: See sim_actuator_packet_t below
 *     Checksum: uint16_t sum of all bytes except checksum field
 *     Total size: 76 bytes (includes 2 bytes padding after header, 2 at end)
 *
 *     Memory layout (all little-endian):
 *       [0-1]    uint8_t[2]   header ("AA")
 *       [2-3]    <padding>
 *       [4-19]   quaternion   q_eci_to_principal (w, x, y, z)
 *       [20-31]  float3       w_principal (x, y, z)
 *       [32-43]  float3       sun_vector_principal (x, y, z)
 *       [44-55]  float3       magdrv_requested (x, y, z)
 *       [56-71]  float[4]     w_reaction_wheels
 *       [72-73]  uint16_t     checksum
 *       [74-75]  <padding>
 *
 *   Transport: USB CDC serial (stdout/stdin), binary mode
 */

#pragma once

#include "linalg.h"
#include "slate.h"
#include <stdint.h>

using namespace linalg::aliases;

#ifdef SIMULATION

// Packet structure for sensor data from simulator to flight computer
typedef struct
{
    // Header
    uint8_t header[2]; // "SS" (0x53 0x53)
    float sim_mjd;     // MJD according to simulator w/ fractional day

    // IMU data
    float3 w_body_raw; // [rad/s] in body frame

    // Magnetometer data
    float3 b_field_local; // B field in body frame (unit vector)

    // Sun sensor data (16 sensors)
    uint16_t sun_sensors[16]; // [0-4095] ADC values

    // GPS data
    float gps_lat;   // [degrees] N+ S-
    float gps_lon;   // [degrees] E+ W-
    float gps_alt;   // [km]
    float gps_time;  // [HHMMSS]
    float3 UTC_date; // [year, month, day]

    // Power monitoring
    float adcs_power;   // [W]
    float adcs_voltage; // [V]
    float adcs_current; // [A]

    // Footer
    uint16_t checksum; // Sum of all bytes
} sim_sensor_packet_t;

// Packet structure for actuator commands from flight computer to simulator
typedef struct
{
    // Header
    uint8_t header[2]; // "AA" (0x41 0x41)

    // Attitude state
    quaternion q_eci_to_principal; // unit quaternion in principal axes frame
    float3 w_principal;            // [rad/s] in principal axes frame
    float3 sun_vector_principal;   // unit vector in body axes frame

    // Magnetorquer requests
    float3 magdrv_requested; // [-1.0 to 1.0] in body frame

    // Reaction wheel requests (4 wheels)
    float w_reaction_wheels[4]; // [rad/s]

    // Footer
    uint16_t checksum; // Sum of all bytes
} sim_actuator_packet_t;

/**
 * Initialize simulation interface
 *
 * Sets up USB serial communication and internal buffers
 */
void sim_init();

/**
 * Read sensor data from simulator and populate slate
 *
 * Blocking with timeout. Syncs on "SS" header, reads full packet,
 * verifies checksum, then populates slate with sensor data.
 *
 * @param slate Pointer to slate structure to populate
 * @param timeout_ms Maximum wait time in milliseconds (0 = non-blocking)
 * @return true if valid packet received, false on timeout or checksum fail
 */
bool sim_read_sensors(slate_t *slate, uint32_t timeout_ms);

/**
 * Send actuator commands to simulator
 *
 * Builds packet with "AA" header, copies actuator requests and attitude state
 * from slate, calculates checksum, and sends binary packet via stdout.
 *
 * @param slate Pointer to slate containing actuator requests and state
 */
void sim_send_actuators(slate_t *slate);

/**
 * Get current time using simulator's MJD-based time
 *
 * Converts simulator MJD time to system absolute_time_t using the t0 offset
 * stored in slate. Must call sim_read_sensors at least once to initialize.
 *
 * @param slate Pointer to slate containing simulation time state
 * @return Current absolute time based on simulator MJD
 */
absolute_time_t sim_get_absolute_time(slate_t *slate);

/**
 * Create a timeout time in the future using simulator time
 *
 * @param slate Pointer to slate containing simulation time state
 * @param ms Milliseconds in the future
 * @return Absolute time ms milliseconds from now in simulator time
 */
absolute_time_t sim_make_timeout_time_ms(slate_t *slate, uint32_t ms);

#endif // SIMULATION
