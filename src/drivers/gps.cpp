/**
 * @author Lundeen Cahilly
 * @date 2025-06-05
 *
 * Simple UART driver for GPS NMEA sentences
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
#define GPS_UART_BAUD 9600
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
            buffer_index = 0;
            sentence_ready = false;
        }

        // Add character to buffer if there's space
        if (buffer_index < GPS_BUFFER_SIZE - 1)
        {
            gps_buffer[buffer_index++] = c;

            // Check for sentence end (CR or LF)
            if (c == '\r' || c == '\n')
            {
                if (buffer_index > 5)
                { // Minimum valid sentence length
                    gps_buffer[buffer_index] = '\0';
                    sentence_ready = true;
                }
            }
        }
        else
        {
            // Buffer overflow, reset
            buffer_index = 0;
            sentence_ready = false;
        }
    }
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
 * Parse GGA sentence for position and altitude data
 */
static void parse_gga_sentence(const char *sentence)
{
    char *token;
    char *sentence_copy = strdup(sentence);
    int field = 0;

    token = strtok(sentence_copy, ",");

    while (token != NULL && field < 15)
    {
        switch (field)
        {
            case 1: // Time (HHMMSS)
                if (strlen(token) >= 6)
                {
                    current_gps_data.timestamp = (uint32_t)atol(token);
                }
                break;
            case 2: // Latitude
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
            case 4: // Longitude
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
            case 6: // Fix quality (0=invalid, 1=GPS, 2=DGPS)
                current_gps_data.valid = (atoi(token) > 0);
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
    // Check for GGA sentence (Global Positioning System Fix Data)
    if (strncmp(sentence, "$GPGGA", 6) == 0 ||
        strncmp(sentence, "$GNGGA", 6) == 0)
    {
        parse_gga_sentence(sentence);
    }
    // Add other sentence types as needed (RMC, etc.)
}

/**
 * Initialize GPS UART interface
 */
void gps_init(void)
{
    // Enable GPS power
    gpio_init(SAMWISE_ADCS_EN_GPS);
    gpio_set_dir(SAMWISE_ADCS_EN_GPS, GPIO_OUT);
    gpio_put(SAMWISE_ADCS_EN_GPS, 1); // Enable GPS power
    sleep_ms(100);                    // Allow GPS to power up

    // Initialize UART
    uart_init(SAMWISE_ADCS_GPS_UART, GPS_UART_BAUD);

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
    current_gps_data.satellites = 0;
    current_gps_data.timestamp = 0;

    LOG_INFO("GPS UART initialized at %d baud", GPS_UART_BAUD);
}

/**
 * Get latest GPS data
 */
bool gps_get_data(gps_data_t *data)
{
    if (!data)
    {
        return false;
    }

    // Process any pending sentences
    if (sentence_ready)
    {
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