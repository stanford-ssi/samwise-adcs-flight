/**
 * @author Lundeen Cahilly
 * @date 2025-05-30
 *
 * This file contains functions for reading magnetometer data from the RM3100
 * using SPI on a RP2350 chip
 */
#pragma once

#include "linalg.h"
using namespace linalg::aliases;

typedef struct magnetometer_data {
    // Magnetometer
    bool magnetometer_alive;
    bool magnetometer_data_valid;     // Flag for magnetometer data validity
    absolute_time_t b_body_read_time; // Time of read [millisecond]
    float3 b_body;     // calibrated magnetic field unit vector in body frame
                       // [unitless]
    float3 b_body_raw; // uncalibrated magnetic field in body frame [microtesla]
    absolute_time_t last_mag_read_start;
} magnetometer_data_t;

// Error codes
typedef enum
{
    RM3100_OK = 0,
    RM3100_ERROR_SPI_COMM,
    RM3100_ERROR_WRONG_CHIP_ID,
    RM3100_ERROR_CONFIG_FAILED,
    RM3100_ERROR_NO_DATA_READY,
    RM3100_ERROR_INVALID_B_BODY_PARAM,
    RM3100_ERROR_INVALID_B_BODY_RAW_PARAM
} rm3100_error_t;

/**
 * Initialize RM3100 magnetometer
 *
 * This function initializes the SPI interface and configures the RM3100
 * magnetometer for continuous measurement mode. It verifies chip presence
 * by reading the revision ID register.
 *
 * @return rm3100_error_t Error code (RM3100_OK on success)
 */
rm3100_error_t rm3100_init(void);

/**
 * Get magnetometer reading
 *
 * This function reads the latest magnetometer data from the RM3100.
 * It checks if new data is available, converts raw readings to
 * engineering units (microTesla), and applies calibration corrections.
 *
 * @param mag_field_body Pointer to float3 vector to store body-aligned magnetic
 * field unit vector
 * @param mag_field_raw Pointer to float3 vector to store calibrated,
 * body-aligned magnetic field in nanoteslas
 * @return rm3100_error_t Error code (RM3100_OK on success)
 */
rm3100_error_t rm3100_get_reading(float3 *mag_field_body,
                                  float3 *mag_field_raw);
