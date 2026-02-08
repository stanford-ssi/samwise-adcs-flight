/**
 * @author Lundeen Cahilly
 * @date 2025-05-30
 *
 * This file contains functions for reading magnetometer data from the RM3100
 * using SPI on a RP2350 chip
 */

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#include "params.h"
#include "macros.h"
#include "magnetometer.h"
#include "pins.h"

#include "linalg.h"
using namespace linalg::aliases;

// RM3100 Register Map
#define RM3100_REG_POLL (0x00)   // Single Measurement Trigger
#define RM3100_REG_CMM (0x01)    // Continuous Measurement Mode
#define RM3100_REG_CCX (0x04)    // Cycle Count X Register
#define RM3100_REG_CCY (0x06)    // Cycle Count Y Register
#define RM3100_REG_CCZ (0x08)    // Cycle Count Z Register
#define RM3100_REG_TMRC (0x0B)   // Continuous Mode Data Rate
#define RM3100_REG_MX (0x24)     // Measurement Results X
#define RM3100_REG_MY (0x27)     // Measurement Results Y
#define RM3100_REG_MZ (0x2A)     // Measurement Results Z
#define RM3100_REG_STATUS (0x34) // Status Register
#define RM3100_REG_REVID (0x36)  // Hardware Revision ID

// RM3100 Configuration Constants
#define RM3100_REVID (0x22)
#define RM3100_CYCLE_COUNT                                                     \
    (200) // Cycle count for all axes (affects resolution/speed)
#define RM3100_CMM_RATE_75_HZ (0x05)
#define RM3100_CMM_RATE_MSB (0x90)
#define RM3100_DRDM_ALL_AXES (0x02)
#define RM3100_LSB_PER_UT (75.0f) // LSB per microTesla at CC=200

// SPI Configuration
#define RM3100_SPI_FREQ (1000 * 1000) // 1 MHz SPI clock
#define RM3100_SPI_TIMEOUT_MS (100)

// Private SPI helper functions
static inline void rm3100_cs_select(void)
{
    gpio_put(SAMWISE_ADCS_SCS_MAGMETER, 0);
}

static inline void rm3100_cs_deselect(void)
{
    gpio_put(SAMWISE_ADCS_SCS_MAGMETER, 1);
}

static bool rm3100_spi_write_reg(uint8_t reg, const uint8_t *data, size_t len)
{
    rm3100_cs_select();

    // Send register address (write command)
    int ret = spi_write_blocking(SAMWISE_ADCS_MAGMETER_SPI, &reg, 1);
    if (ret != 1)
    {
        rm3100_cs_deselect();
        return false;
    }

    // Send data
    ret = spi_write_blocking(SAMWISE_ADCS_MAGMETER_SPI, data, len);
    rm3100_cs_deselect();

    return ret == (int)len;
}

static bool rm3100_spi_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t cmd = reg | 0x80; // Set MSB for read command

    rm3100_cs_select();

    // Send register address (read command)
    int ret = spi_write_blocking(SAMWISE_ADCS_MAGMETER_SPI, &cmd, 1);
    if (ret != 1)
    {
        LOG_DEBUG("[rm3100] Failed to send read command");
        rm3100_cs_deselect();
        return false;
    }

    // Read data
    ret = spi_read_blocking(SAMWISE_ADCS_MAGMETER_SPI, 0, data, len);
    rm3100_cs_deselect();

    return ret == (int)len;
}

/**
 * Apply magnetometer calibration
 *
 * This function applies hard iron offset correction and soft iron matrix
 * transformation to raw magnetometer readings. The calibration compensates
 * for magnetic distortions in the sensor environment (such as other sensors and
 * boards).
 *
 * @param raw_reading Raw magnetometer reading
 * @param calibrated_reading Pointer to store calibrated result in microTesla
 */
static void rm3100_apply_calibration(const float3 &raw_reading,
                                     float3 *calibrated_reading)
{
    if (calibrated_reading == NULL)
    {
        return;
    }

    // Step 1: Apply hard iron offset correction
    float3 centered = raw_reading - MAG_HARD_IRON_OFFSET;

    // Step 2: Apply soft iron correction matrix transformation
    *calibrated_reading = mul(MAG_SOFT_IRON_MATRIX, centered);
}

/**
 * Initialize RM3100 magnetometer
 *
 * This function initializes the SPI interface and configures the RM3100
 * magnetometer for continuous measurement mode. It verifies chip presence
 * by reading the revision ID register.
 *
 * @return rm3100_error_t Error code (RM3100_OK on success)
 */
rm3100_error_t rm3100_init(void)
{
    // Enable magnetometer power FIRST
    gpio_init(SAMWISE_ADCS_EN_MAGMETER);
    gpio_set_dir(SAMWISE_ADCS_EN_MAGMETER, GPIO_OUT);
    gpio_put(SAMWISE_ADCS_EN_MAGMETER, 0); // Enable power

    // Give it time to power up
    sleep_ms(100);

    // Initialize SPI interface
    spi_init(SAMWISE_ADCS_MAGMETER_SPI, RM3100_SPI_FREQ);
    spi_set_format(SAMWISE_ADCS_MAGMETER_SPI, 8, SPI_CPOL_0, SPI_CPHA_0,
                   SPI_MSB_FIRST);

    // Configure SPI pins
    gpio_set_function(SAMWISE_ADCS_SCLK_MAGMETER, GPIO_FUNC_SPI);
    gpio_set_function(SAMWISE_ADCS_MOSI_MAGMETER, GPIO_FUNC_SPI);
    gpio_set_function(SAMWISE_ADCS_MISO_MAGMETER, GPIO_FUNC_SPI);

    // Configure CS pin manually
    gpio_init(SAMWISE_ADCS_SCS_MAGMETER);
    gpio_set_dir(SAMWISE_ADCS_SCS_MAGMETER, GPIO_OUT);
    gpio_put(SAMWISE_ADCS_SCS_MAGMETER, 1); // Start deselected

    // Verify chip presence by reading revision ID
    uint8_t revision_id;
    if (!rm3100_spi_read_reg(RM3100_REG_REVID, &revision_id, 1))
    {
        LOG_INFO("[rm3100] SPI communication failed during chip ID read");
        return RM3100_ERROR_SPI_COMM;
    }

    if (revision_id != RM3100_REVID)
    {
        LOG_INFO("[rm3100] Wrong chip ID. Expected 0x%02X, got 0x%02X",
                 RM3100_REVID, revision_id);
        return RM3100_ERROR_WRONG_CHIP_ID;
    }

    LOG_INFO("[rm3100] Chip ID verified (0x%02X)", revision_id);

    // Configure cycle counts for all axes (affects resolution and measurement
    // time)
    uint8_t cycle_counts[6] = {
        0, RM3100_CYCLE_COUNT, // X-axis: MSB, LSB
        0, RM3100_CYCLE_COUNT, // Y-axis: MSB, LSB
        0, RM3100_CYCLE_COUNT  // Z-axis: MSB, LSB
    };

    if (!rm3100_spi_write_reg(RM3100_REG_CCX, cycle_counts, 6))
    {
        LOG_INFO("[rm3100] Failed to configure cycle counts");
        return RM3100_ERROR_CONFIG_FAILED;
    }

    // Set continuous measurement mode data rate (75 Hz)
    uint8_t data_rate = RM3100_CMM_RATE_75_HZ | RM3100_CMM_RATE_MSB;
    if (!rm3100_spi_write_reg(RM3100_REG_TMRC, &data_rate, 1))
    {
        LOG_INFO("[rm3100] Failed to configure data rate");
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
        LOG_INFO("[rm3100] Failed to enable continuous measurement mode");
        return RM3100_ERROR_CONFIG_FAILED;
    }

    LOG_INFO(
        "[rm3100] Initialization successful, continuous mode enabled at 75 Hz");
    return RM3100_OK;
}

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
rm3100_error_t rm3100_get_reading(float3 *mag_field_body, float3 *mag_field_raw)
{
    // Null pointer checks
    if (mag_field_body == NULL)
    {
        return RM3100_ERROR_INVALID_B_BODY_PARAM;
    }
    if (mag_field_raw == NULL)
    {
        return RM3100_ERROR_INVALID_B_BODY_RAW_PARAM;
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
    int32_t raw_x_lsb = ((int32_t)raw_data[0] << 16) |
                        ((int32_t)raw_data[1] << 8) | raw_data[2];
    int32_t raw_y_lsb = ((int32_t)raw_data[3] << 16) |
                        ((int32_t)raw_data[4] << 8) | raw_data[5];
    int32_t raw_z_lsb = ((int32_t)raw_data[6] << 16) |
                        ((int32_t)raw_data[7] << 8) | raw_data[8];

    // Sign extend from 24-bit to 32-bit
    if (raw_x_lsb & 0x800000)
        raw_x_lsb |= 0xFF000000;
    if (raw_y_lsb & 0x800000)
        raw_y_lsb |= 0xFF000000;
    if (raw_z_lsb & 0x800000)
        raw_z_lsb |= 0xFF000000;

    // Convert from LSB to microTesla using scale factor
    const float lsb_to_uT = 1.0f / RM3100_LSB_PER_UT;

    // 1. Scale
    // Axis conversion: [x,y,z] magmeter -> [x, -y, -z]
    // TODO: verify for flight. the current value is for ruby chan board (green
    // w/ face) The sensor needs to be negated on X and Y axes to match body
    // frame
    *mag_field_raw = {raw_x_lsb * lsb_to_uT, raw_y_lsb * lsb_to_uT,
                      raw_z_lsb * lsb_to_uT};

    // mag_field_raw should contain the uncalibrated raw reading
    // (already set above, no calibration needed)

    LOG_DEBUG("[rm3100] b_body_raw = [%.6f, %.6f, %.6f]", mag_field_raw->x,
              mag_field_raw->y, mag_field_raw->z);

    // Apply calibration to get true magnetic field reading, compensating for
    // magnetic fields generated by the electronics on the spacecraft
    float3 calibrated_field;
    rm3100_apply_calibration(*mag_field_raw, &calibrated_field);

    // Normalize to get unit vector in body frame
    *mag_field_body = normalize(calibrated_field);
    mag_field_body->x = -mag_field_body->x;
    mag_field_body->z = -mag_field_body->z;
    return RM3100_OK;
}