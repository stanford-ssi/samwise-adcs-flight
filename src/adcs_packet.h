/**
 * @author Niklas Vainio
 * @date 2025-05-27
 *
 * This file defines the ADCS telemetry struct.
 * IMPORTANT: KEEP UP TO DATE WITH THE PICUBED
 *
 * Last updated: 05/27/2025
 *
 * (this must live in a separate file for C include reasons)
 */

#pragma once

#include <stdint.h>

typedef struct __attribute__((packed))
{
    // Angular velocity
    float w;

    // Quaternion
    float q0, q1, q2, q3;

    // Time
    float mjd;
    float UTC_time;

    // Power
    float voltage;
    float current;

    // Statekeeping
    uint8_t state;
    uint32_t boot_counter;
} adcs_packet_t;
