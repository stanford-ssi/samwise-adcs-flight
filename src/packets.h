/**
 * @author Niklas Vainio, Lundeen Cahilly
 * @date 2025-05-27
 *
 * This file defines the ADCS telemetry structs.
 * IMPORTANT: KEEP UP TO DATE WITH THE PICUBED AND MOTOR BOARD
 *
 * Last updated: 10/25/2025
 *
 * (this must live in a separate file for C include reasons)
 */

#pragma once

#include <stdint.h>

/*
 * Packet sent from ADCS to picubed
 */
typedef struct __attribute__((packed))
{
    // Angular velocity
    float w;

    // Quaternion Estimate
    float q0, q1, q2, q3;

    // Misc Data
    char state;
    uint32_t boot_count;

} adcs_packet_t;

/*
 * Packet sent to motor board
 */
typedef struct
{
    bool reaction_wheels_enabled[4];
    float w_reaction_wheels_requested[4];
    uint32_t checksum;
} motor_packet_tx_t;

/*
 * Packet received from motor board
 */
typedef struct
{
    float motor_board_voltage;
    float motor_board_current;
    bool rw_alive[4];
    bool rw_data_valid[4];
    bool w_rw[4]; // actual reaction wheel speeds
    bool magnetometer_alive;
    bool magnetometer_data_valid;
    float b_body_raw[3];
    uint32_t checksum;
} motor_packet_rx_t;