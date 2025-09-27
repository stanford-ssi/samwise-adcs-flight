/**
 * @author Lundeen Cahilly
 * @date 2025-09-25
 *
 * UART driver to read GPS NMEA sentences (RMC)
 */

#include "gps.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "macros.h"
#include "pico/stdlib.h"
#include "pins.h"
#include <stdlib.h>
#include <string.h>

// UART configuration
#define GPS_UART_BAUD 115200
#define GPS_UART_DATA_BITS 8
#define GPS_UART_STOP_BITS 1
#define GPS_UART_PARITY UART_PARITY_NONE

// Buffer for incoming NMEA sentences
static char gps_buffer[GPS_BUFFER_SIZE];
static volatile uint16_t buffer_index = 0;
static volatile bool sentence_ready = false;
static gps_data_t current_gps_data = {0};

/**
 * UART interrupt handler
 * Receives characters and builds NMEA sentences
 */
void gps_uart_irq_handler(void)
{
    while (uart_is_readable(SAMWISE_ADCS_GPS_UART))
    {
        char c = uart_getc(SAMWISE_ADCS_GPS_UART);

        // Handle sentence start
        if (c == '$')
        {
            if (!sentence_ready)
            {
                buffer_index = 0;
            }
            // Don't clear sentence_ready here!
        }

        // Add character to buffer if there's space
        if (buffer_index < GPS_BUFFER_SIZE - 1)
        {
            gps_buffer[buffer_index] = c;

            // Check for sentence end (CR or LF)
            if (c == '\r' || c == '\n')
            {
                gps_buffer[buffer_index] = '\0'; // Null terminate
                if (buffer_index > 5)
                {
                    sentence_ready = true;
                    // LOG_DEBUG("[GPS] Sentence complete, length: %d",
                    // buffer_index);
                }
                buffer_index = 0; // Reset for next sentence
            }
            else
            {
                buffer_index++;
            }
        }
        else
        {
            // LOG_DEBUG("[GPS] Buffer overflow, resetting");
            buffer_index = 0;
            sentence_ready = false;
        }
    }
}

/**
 * Validate NMEA sentence checksum
 */
static bool validate_nmea_checksum(const char *sentence)
{

    if (!sentence || strlen(sentence) < 8)
    {
        return false;
    }

    // Find the asterisk
    const char *asterisk = strrchr(sentence, '*');
    if (!asterisk || strlen(asterisk) != 3)
    {
        return false;
    }

    // Calculate checksum (XOR all characters between $ and *)
    uint8_t calculated_checksum = 0;
    for (const char *p = sentence + 1; p < asterisk; p++)
    {
        calculated_checksum ^= *p;
    }

    // Parse received checksum
    char checksum_str[3] = {asterisk[1], asterisk[2], '\0'};
    uint8_t received_checksum = (uint8_t)strtol(checksum_str, NULL, 16);

    bool valid = (calculated_checksum == received_checksum);

    if (valid)
    {
        LOG_DEBUG("[gps] Checksum matches: calc=0x%02X, recv=0x%02X",
                  calculated_checksum, received_checksum);
    }

    return valid;
}

/**
 * Validate coordinate is within valid GPS range
 */
static bool validate_coordinate(float lat, float lon)
{
    return (lat >= -90.0f && lat <= 90.0f && lon >= -180.0f && lon <= 180.0f);
}

/**
 * Parse NMEA coordinate (DDMM.MMMM or DDDMM.MMMM) to decimal degrees
 */
static float parse_coordinate(const char *coord_str, const char *direction)
{
    if (!coord_str || !direction || strlen(coord_str) == 0)
    {
        return 0.0f;
    }

    float coord = atof(coord_str);
    int degrees = (int)(coord / 100.0f);
    float minutes = coord - (degrees * 100.0f);
    float decimal_degrees = degrees + (minutes / 60.0f);

    // Apply direction (negative for South/West)
    if (direction[0] == 'S' || direction[0] == 'W')
    {
        decimal_degrees = -decimal_degrees;
    }

    return decimal_degrees;
}

/**
 * Parse RMC sentence for position, time, and date data
 */
static void parse_rmc_sentence(const char *sentence)
{
    char *token;
    char *sentence_copy = strdup(sentence);
    if (sentence_copy == NULL)
    {
        // Handle memory allocation failure
        return;
    }
    int field = 0;

    token = strtok(sentence_copy, ",");

    while (token != NULL && field < 12)
    {
        switch (field)
        {
            case 1: // Time (HHMMSS)
                if (strlen(token) >= 6)
                {
                    current_gps_data.timestamp = (uint32_t)atol(token);
                }
                break;
            case 2: // Status (A=active, V=void)
                current_gps_data.valid = (token[0] == 'A');
                break;
            case 3: // Latitude
                if (strlen(token) > 0)
                {
                    char *next_token = strtok(NULL, ",");
                    if (next_token)
                    {
                        current_gps_data.latitude =
                            parse_coordinate(token, next_token);
                        field++; // Skip direction field
                    }
                }
                break;
            case 5: // Longitude
                if (strlen(token) > 0)
                {
                    char *next_token = strtok(NULL, ",");
                    if (next_token)
                    {
                        current_gps_data.longitude =
                            parse_coordinate(token, next_token);
                        field++; // Skip direction field
                    }
                }
                break;
            case 7: // Speed over ground (knots)
                if (strlen(token) > 0)
                {
                    current_gps_data.speed = atof(token);
                }
                break;
            case 8: // Course over ground (degrees)
                if (strlen(token) > 0)
                {
                    current_gps_data.course = atof(token);
                }
                break;
            case 9: // Date (DDMMYY)
                if (strlen(token) >= 6)
                {
                    current_gps_data.date = (uint32_t)atol(token);
                }
                break;
        }

        token = strtok(NULL, ",");
        field++;
    }

    free(sentence_copy);
}

/**
 * Parse GGA sentence for position and altitude data
 */
static void parse_gga_sentence(const char *sentence)
{
    char *token;
    char *sentence_copy = strdup(sentence);
    if (sentence_copy == NULL)
    {
        // Handle memory allocation failure
        return;
    }
    int field = 0;

    token = strtok(sentence_copy, ",");

    while (token != NULL && field < 15)
    {
        switch (field)
        {
            case 1: // Time (HHMMSS) - only update if RMC hasn't set it
                if (strlen(token) >= 6 && current_gps_data.timestamp == 0)
                {
                    current_gps_data.timestamp = (uint32_t)atol(token);
                }
                break;
            case 2: // Latitude - only update if RMC hasn't set it
                if (strlen(token) > 0 && current_gps_data.latitude == 0.0f)
                {
                    char *next_token = strtok(NULL, ",");
                    if (next_token)
                    {
                        current_gps_data.latitude =
                            parse_coordinate(token, next_token);
                        field++; // Skip direction field
                    }
                }
                break;
            case 4: // Longitude - only update if RMC hasn't set it
                if (strlen(token) > 0 && current_gps_data.longitude == 0.0f)
                {
                    char *next_token = strtok(NULL, ",");
                    if (next_token)
                    {
                        current_gps_data.longitude =
                            parse_coordinate(token, next_token);
                        field++; // Skip direction field
                    }
                }
                break;
            case 6: // Fix quality (0=invalid, 1=GPS, 2=DGPS) - RMC takes
                    // precedence
                if (!current_gps_data.valid)
                {
                    current_gps_data.valid = (atoi(token) > 0);
                }
                break;
            case 7: // Number of satellites
                current_gps_data.satellites = (uint8_t)atoi(token);
                break;
            case 9: // Altitude above sea level
                if (strlen(token) > 0)
                {
                    current_gps_data.altitude = atof(token);
                }
                break;
        }

        token = strtok(NULL, ",");
        field++;
    }

    free(sentence_copy);
}

/**
 * Process received NMEA sentence
 */
static void process_nmea_sentence(const char *sentence)
{
    // Check for RMC sentence
    if (strncmp(sentence, "$GPRMC", 6) == 0 ||
        strncmp(sentence, "$GNRMC", 6) == 0)
    {
        // Validate checksum for RMC sentences
        if (!validate_nmea_checksum(sentence))
        {
            return;
        }
        LOG_DEBUG("[gps] Processing RMC: %s", sentence);
        parse_rmc_sentence(sentence);
    }

    // Check for GGA sentence
    if (strncmp(sentence, "$GPGGA", 6) == 0 ||
        strncmp(sentence, "$GNGGA", 6) == 0)
    {
        // Validate checksum for GGA sentences
        if (!validate_nmea_checksum(sentence))
        {
            return;
        }
        LOG_DEBUG("[gps] Processing GGA: %s", sentence);
        parse_gga_sentence(sentence);
    }
}

/**
 * Initialize GPS UART interface
 * @return true if GPS initialization successful, false otherwise
 */
bool gps_init(void)
{
    // Enable GPS power
    gpio_init(SAMWISE_ADCS_EN_GPS);
    gpio_set_dir(SAMWISE_ADCS_EN_GPS, GPIO_OUT);
    gpio_put(SAMWISE_ADCS_EN_GPS, 0); // Pull low to enable GPS
    sleep_ms(1000);

    // Initialize UART
    uint actual_baud = uart_init(SAMWISE_ADCS_GPS_UART, GPS_UART_BAUD);
    if (actual_baud == 0)
    {
        LOG_ERROR("[gps] Failed to initialize GPS UART");
        return false;
    }

    // Set GPIO functions for UART
    gpio_set_function(
        SAMWISE_ADCS_TX_TO_GPS,
        UART_FUNCSEL_NUM(SAMWISE_ADCS_GPS_UART, SAMWISE_ADCS_TX_TO_GPS));
    gpio_set_function(
        SAMWISE_ADCS_RX_FROM_GPS,
        UART_FUNCSEL_NUM(SAMWISE_ADCS_GPS_UART, SAMWISE_ADCS_RX_FROM_GPS));

    uart_set_format(SAMWISE_ADCS_GPS_UART, GPS_UART_DATA_BITS,
                    GPS_UART_STOP_BITS, GPS_UART_PARITY);

    // Set GPIO functions for UART
    gpio_set_function(
        SAMWISE_ADCS_TX_TO_GPS,
        UART_FUNCSEL_NUM(SAMWISE_ADCS_GPS_UART, SAMWISE_ADCS_TX_TO_GPS));
    gpio_set_function(
        SAMWISE_ADCS_RX_FROM_GPS,
        UART_FUNCSEL_NUM(SAMWISE_ADCS_GPS_UART, SAMWISE_ADCS_RX_FROM_GPS));

    // Set data format
    uart_set_format(SAMWISE_ADCS_GPS_UART, GPS_UART_DATA_BITS,
                    GPS_UART_STOP_BITS, GPS_UART_PARITY);

    // Enable UART interrupt
    irq_set_exclusive_handler(UART_IRQ_NUM(SAMWISE_ADCS_GPS_UART),
                              gps_uart_irq_handler);
    irq_set_enabled(UART_IRQ_NUM(SAMWISE_ADCS_GPS_UART), true);
    uart_set_irq_enables(SAMWISE_ADCS_GPS_UART, true,
                         false); // RX interrupt only

    // Initialize GPS data structure
    current_gps_data.valid = false;
    current_gps_data.latitude = 0.0f;
    current_gps_data.longitude = 0.0f;
    current_gps_data.altitude = 0.0f;
    current_gps_data.timestamp = 0;
    current_gps_data.date = 0;
    current_gps_data.speed = 0.0f;
    current_gps_data.course = 0.0f;
    current_gps_data.satellites = 0;

    LOG_INFO("[gps] UART initialized at %d baud (actual: %d)", GPS_UART_BAUD,
             actual_baud);

    return true;
}

/**
 * Get latest GPS data
 */
bool gps_get_data(gps_data_t *data)
{
    if (!data)
        return false;

    // LOG_DEBUG("[GPS] gps_get_data called, sentence_ready = %s",
    //           sentence_ready ? "true" : "false");

    // Process any pending sentences
    if (sentence_ready)
    {
        // LOG_DEBUG("[GPS] About to process sentence");
        process_nmea_sentence(gps_buffer);
        sentence_ready = false;
    }

    // Copy current data
    *data = current_gps_data;
    return current_gps_data.valid;
}

/**
 * Check if new NMEA sentence is available
 */
bool gps_sentence_available(void)
{
    return sentence_ready;
}