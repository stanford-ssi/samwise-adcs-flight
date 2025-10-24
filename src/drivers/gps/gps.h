/**
 * @author Lundeen Cahilly
 * @date 2025-06-05
 *
 * Simple UART driver for GPS NMEA sentences
 */

#pragma once

#include "linalg.h"
using namespace linalg::aliases;

#define GPS_BUFFER_SIZE 256
#define GPS_MAX_SENTENCE_LENGTH 82 // NMEA standard max length
#define GPS_DATA_EXPIRATION_MS 1 // At 7.6 km/s, gives 

typedef struct
{
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