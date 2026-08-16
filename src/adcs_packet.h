/**
 * @author Niklas Vainio
 * @date 2025-05-27
 *
 * This file defines the ADCS telemetry struct.
 * IMPORTANT: KEEP UP TO DATE WITH THE PICUBED
 *
 * Last updated: 07/25/2026 - grew 41 -> 77 bytes. The PiCubed decoder must be
 * updated in lockstep or every packet decodes as garbage.
 *
 * Adding fields is not free: send_msg() sizes its COBS buffer as len + 2, where
 * len = 7 + sizeof(adcs_packet_t). At 77 bytes that is an exact fit.
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

    // Body frame sensor measurements.
    // 
    // We use these to validate the attitude estimate from the MEKF by running an independent 
    // TRIAD attitude estimate.
    // 
    // We can also use these to validate the sensor health by comparing the measured values
    // to the expected values, given the EKF's attitude estimate.
    // * sun_resid = angle( q ⊗ sun_eci(mjd, utc_time) ,  sun_body )
    // * mag_resid = angle( q ⊗ mag_eci(lat, lon, alt) ,  mag_body )
    float sun_body_x, sun_body_y, sun_body_z;
    float mag_body_x, mag_body_y, mag_body_z;

    // Longitude, latitude, and altitude of the satellite.
    // NOTE: longitude first. Easy to transpose on the ground side.
    float lon, lat, alt;

    // Statekeeping
    uint8_t state;
    uint32_t boot_counter;
} adcs_packet_t;

// Pin the wire size so any future field addition fails loudly here rather than
// silently desynchronising from the PiCubed decoder.
#ifdef __cplusplus
static_assert(sizeof(adcs_packet_t) == 77,
              "adcs_packet_t size changed - update the PiCubed decoder");
#endif
