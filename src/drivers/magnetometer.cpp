/**
 * @author Lundeen Cahilly
 * @date 2025-05-29
 *
 * This file contains functions for reading magnetometer data from the RM3100
 * using SPI on a RP2350 chip
 */

#include "magnetometer.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "linalg.h" // For float3 type and math functions
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include <cmath>
#include <cstring>
#include <stdio.h>

// Pin definitions for SPI - TODO: Update for ADCS board
#define SPI_PORT spi0
#define SPI_SCK 2  // GPIO 2 - SCK
#define SPI_MOSI 3 // GPIO 3 - MOSI
#define SPI_MISO 4 // GPIO 4 - MISO
#define SPI_CS 5   // GPIO 5 - CS

// RM3100 Register Map
#define RM3100_REG_POLL 0x00   // Single Measurement Trigger
#define RM3100_REG_CMM 0x01    // Continuous Measurement Mode
#define RM3100_REG_CCX 0x04    // Cycle Count X Register
#define RM3100_REG_CCY 0x06    // Cycle Count Y Register
#define RM3100_REG_CCZ 0x08    // Cycle Count Z Register
#define RM3100_REG_TMRC 0x0B   // Continuous Mode Data Rate
#define RM3100_REG_MX 0x24     // Measurement Results X
#define RM3100_REG_MY 0x27     // Measurement Results Y
#define RM3100_REG_MZ 0x2A     // Measurement Results Z
#define RM3100_REG_STATUS 0x34 // Status Register
#define RM3100_REG_REVID 0x36  // Hardware Revision ID

// RM3100 Configuration Constants
#define RM3100_REVID 0x22
#define RM3100_CYCLE_COUNT                                                     \
    200 // Cycle count for all axes (affects resolution/speed)
#define RM3100_CMM_RATE_75_HZ 0x05
#define RM3100_CMM_RATE_MSB 0x90
#define RM3100_DRDM_ALL_AXES 0x02
#define RM3100_LSB_PER_UT 75.0f // LSB per microTesla at CC=200

// Error codes
typedef enum
{
    RM3100_OK = 0,
    RM3100_ERROR_SPI_COMM,
    RM3100_ERROR_WRONG_CHIP_ID,
    RM3100_ERROR_CONFIG_FAILED,
    RM3100_ERROR_NO_DATA_READY,
    RM3100_ERROR_INVALID_PARAM
} rm3100_error_t;

// Binary info for picotool
bi_decl(bi_4pins_with_func(SPI_MISO, SPI_MOSI, SPI_SCK, SPI_CS, GPIO_FUNC_SPI));
bi_decl(bi_program_description("RM3100 Magnetometer Flight Driver"));

// Private SPI helper functions
static inline void rm3100_cs_select(void)
{
    gpio_put(SPI_CS, 0);
}

static inline void rm3100_cs_deselect(void)
{
    gpio_put(SPI_CS, 1);
}

static bool rm3100_spi_write_reg(uint8_t reg, const uint8_t *data, size_t len)
{
    rm3100_cs_select();

    // Send register address (write command)
    int ret = spi_write_blocking(SPI_PORT, &reg, 1);
    if (ret != 1)
    {
        rm3100_cs_deselect();
        return false;
    }

    // Send data
    ret = spi_write_blocking(SPI_PORT, data, len);
    rm3100_cs_deselect();

    return ret == (int)len;
}

static bool rm3100_spi_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t cmd = reg | 0x80; // Set MSB for read command

    rm3100_cs_select();

    // Send register address (read command)
    int ret = spi_write_blocking(SPI_PORT, &cmd, 1);
    if (ret != 1)
    {
        rm3100_cs_deselect();
        return false;
    }

    // Read data
    ret = spi_read_blocking(SPI_PORT, 0, data, len);
    rm3100_cs_deselect();

    return ret == (int)len;
}

/**
 * @brief Initialize RM3100 magnetometer
 *
 * This function initializes the SPI interface and configures the RM3100
 * magnetometer for continuous measurement mode. It verifies chip presence
 * by reading the revision ID register.
 *
 * @return rm3100_error_t Error code (RM3100_OK on success)
 */
rm3100_error_t rm3100_init(void)
{
    // Initialize SPI interface
    spi_init(SPI_PORT, 1000 * 1000); // 1 MHz SPI clock
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Configure SPI pins
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);

    // Configure CS pin manually
    gpio_init(SPI_CS);
    gpio_set_dir(SPI_CS, GPIO_OUT);
    gpio_put(SPI_CS, 1); // Start deselected

    // Small delay for SPI to stabilize
    sleep_ms(10);

    // Verify chip presence by reading revision ID
    uint8_t revision_id;
    if (!rm3100_spi_read_reg(RM3100_REG_REVID, &revision_id, 1))
    {
        return RM3100_ERROR_SPI_COMM;
    }

    if (revision_id != RM3100_REVID)
    {
        return RM3100_ERROR_WRONG_CHIP_ID;
    }

    // Configure cycle counts for all axes (affects resolution and measurement
    // time)
    uint8_t cycle_counts[6] = {
        0, RM3100_CYCLE_COUNT, // X-axis: MSB, LSB
        0, RM3100_CYCLE_COUNT, // Y-axis: MSB, LSB
        0, RM3100_CYCLE_COUNT  // Z-axis: MSB, LSB
    };

    if (!rm3100_spi_write_reg(RM3100_REG_CCX, cycle_counts, 6))
    {
        return RM3100_ERROR_CONFIG_FAILED;
    }

    // Set continuous measurement mode data rate (75 Hz)
    uint8_t data_rate = RM3100_CMM_RATE_75_HZ | RM3100_CMM_RATE_MSB;
    if (!rm3100_spi_write_reg(RM3100_REG_TMRC, &data_rate, 1))
    {
        return RM3100_ERROR_CONFIG_FAILED;
    }

    // Enable continuous measurement mode for all axes
    uint8_t cmm_config = (1 << 0) |                    // START bit
                         (RM3100_DRDM_ALL_AXES << 2) | // DRDY after all axes
                         (1 << 4) |                    // CMX enable
                         (1 << 5) |                    // CMY enable
                         (1 << 6);                     // CMZ enable

    if (!rm3100_spi_write_reg(RM3100_REG_CMM, &cmm_config, 1))
    {
        return RM3100_ERROR_CONFIG_FAILED;
    }

    return RM3100_OK;
}

/**
 * @brief Get magnetometer reading
 *
 * This function reads the latest magnetometer data from the RM3100.
 * It checks if new data is available and converts raw readings to
 * engineering units (microTesla).
 *
 * @param mag_field Pointer to float3 vector to store magnetic field in
 * microTesla
 * @return rm3100_error_t Error code (RM3100_OK on success)
 */
rm3100_error_t rm3100_get_reading(float3 *mag_field)
{
    if (mag_field == NULL)
    {
        return RM3100_ERROR_INVALID_PARAM;
    }

    // Check if new data is ready
    uint8_t status;
    if (!rm3100_spi_read_reg(RM3100_REG_STATUS, &status, 1))
    {
        return RM3100_ERROR_SPI_COMM;
    }

    if (!(status & 0x80))
    { // DRDY bit not set
        return RM3100_ERROR_NO_DATA_READY;
    }

    // Read all measurement registers (9 bytes: 3 per axis)
    uint8_t raw_data[9];
    if (!rm3100_spi_read_reg(RM3100_REG_MX, raw_data, 9))
    {
        return RM3100_ERROR_SPI_COMM;
    }

    // Convert 24-bit signed values to 32-bit signed integers
    int32_t raw_x = ((int32_t)raw_data[0] << 16) | ((int32_t)raw_data[1] << 8) |
                    raw_data[2];
    int32_t raw_y = ((int32_t)raw_data[3] << 16) | ((int32_t)raw_data[4] << 8) |
                    raw_data[5];
    int32_t raw_z = ((int32_t)raw_data[6] << 16) | ((int32_t)raw_data[7] << 8) |
                    raw_data[8];

    // Sign extend from 24-bit to 32-bit
    if (raw_x & 0x800000)
        raw_x |= 0xFF000000;
    if (raw_y & 0x800000)
        raw_y |= 0xFF000000;
    if (raw_z & 0x800000)
        raw_z |= 0xFF000000;

    // Convert to microTesla using calibrated scale factor
    const float scale = 1.0f / RM3100_LSB_PER_UT;
    mag_field->x = raw_x * scale;
    mag_field->y = raw_y * scale;
    mag_field->z = raw_z * scale;

    return RM3100_OK;
}

/**
 * @brief Calculate magnitude of magnetic field vector
 *
 * @param mag_field Pointer to float3 magnetic field vector
 * @return float Magnitude in microTesla
 */
float rm3100_magnitude(const float3 *mag_field)
{
    if (mag_field == NULL)
    {
        return 0.0f;
    }

    return sqrtf((mag_field->x * mag_field->x) + (mag_field->y * mag_field->y) +
                 (mag_field->z * mag_field->z));
}

// Example usage
int main()
{
    stdio_init_all();
    sleep_ms(2000); // Allow USB enumeration

    printf("RM3100 Flight Driver Initializing...\n");

    // Initialize magnetometer
    rm3100_error_t init_result = rm3100_init();
    if (init_result != RM3100_OK)
    {
        printf("ERROR: RM3100 initialization failed with code %d\n",
               init_result);
        return -1;
    }

    printf("RM3100 initialized successfully\n");
    printf("Timestamp(ms),M_x(µT),M_y(µT),M_z(µT),M_total(µT)\n");

    float3 mag_field;

    while (true)
    {
        rm3100_error_t result = rm3100_get_reading(&mag_field);

        if (result == RM3100_OK)
        {
            uint64_t timestamp_ms = time_us_64() / 1000;
            float magnitude = rm3100_magnitude(&mag_field);

            printf("%llu,%.3f,%.3f,%.3f,%.3f\n", timestamp_ms, mag_field.x,
                   mag_field.y, mag_field.z, magnitude);
        }
        else if (result != RM3100_ERROR_NO_DATA_READY)
        {
            printf("ERROR: Failed to read magnetometer data, code %d\n",
                   result);
        }

        // sleep_ms(1); // Optional: limit reading rate
    }

    return 0;
}