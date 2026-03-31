/**
 * @author Lundeen Cahilly
 * @date 2025-06-05
 *
 * Simple UART driver for GPS NMEA sentences
 */

#pragma once

#include "pico/time.h"
#include "linalg.h"
using namespace linalg::aliases;

#define GPS_BUFFER_SIZE 256
#define GPS_MAX_SENTENCE_LENGTH 82 // NMEA standard max length

typedef struct {
    bool valid;
    float latitude;     // Decimal degrees
    float longitude;    // Decimal degrees
    float altitude;     // Altitude above sea level in meters (from GGA)
    uint32_t timestamp; // UTC time as HHMMSS
    uint32_t date;      // UTC date as DDMMYY
    float speed;        // Speed over ground in knots
    float course;       // Course over ground in degrees
    uint8_t satellites; // Number of satellites in use (from GGA)
} gps_data_t;

typedef struct {
    // Lightly processed GPS data
    bool gps_alive;
    bool gps_data_valid;
    absolute_time_t gps_read_time;
    float gps_lat;
    float gps_lon;
    float gps_alt;
    float gps_time;
    float gps_speed;
    float gps_course;

    // World state processed from GPS
    float3 UTC_date;
    float UTC_time;
    float MJD;

    // Position and velocity
    float3 r_ecef;
    float3 r_eci;
    float3 v_eci;
} gps_data_processed_t;

/**
 * Initialize GPS UART interface
 * Sets up UART with interrupts for receiving NMEA sentences
 * @return true if initialization was successful, false otherwise
 */
bool gps_init(void);

/**
 * Get latest GPS data
 * @param data Pointer to structure to fill with GPS data
 * @return true if valid data available, false otherwise
 */
bool gps_get_data(gps_data_t *data);

/**
 * Check if new NMEA sentence is available
 * @return true if new sentence ready for processing
 */
bool gps_sentence_available(void);
