/**
 * @author Niklas Vainio
 * @date 2025-05-08
 *
 * This file defines utilities for controlling the BMI270 IMU.
 *
 * This driver was entirely made possibly by the tireless work of Pete Mahowland
 */

#include "imu.h"

#include "constants.h"
#include "macros.h"
#include "pins.h"

#include <stdint.h>
#include <stdlib.h>

#include "external/bmi270_legacy.h"
#include "external/bmi2_defs.h"
#include "linalg.h"
#include "pico/stdlib.h"

using namespace linalg::aliases;

/******************************************************************************/
/*!                 Macro definitions                                         */
#define BMI2XY_SHUTTLE_ID UINT16_C(0x1B8)

#define BMI_EXPECTED_CHIP_ID (0x24)

/*! Macro that defines read write length */
#define READ_WRITE_LEN UINT8_C(46)

/*! I2C Device address */
#define BMI_I2C_DEVICE_ADDRESS (BMI2_I2C_PRIM_ADDR)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C or SPI bus instance */
static uint8_t bus_inst;

/*! Structure to hold interface configurations */
static struct adcs_intf_config intf_conf;

static struct bmi2_dev bmi;

/******************************************************************************/

/**
 * Thin wrapper around writing to the I2C bus
 */
static int8_t imu_write_i2c(uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    // LOG_INFO("Writing %d bytes to address 0x%x", count, reg_addr);

    int num_bytes_read = 0;
    uint8_t msg[count + 1];
    // Check to make sure caller is sending 1 or more byte
    if (count < 1)
    {
        return 0;
    }
    // Append register address to front of data packet
    msg[0] = reg_addr;
    for (int i = 0; i < count; i++)
    {
        msg[i + 1] = reg_data[i];
    }
    // Write data to register(s) over I2C
    i2c_write_blocking(SAMWISE_ADCS_IMU_I2C, BMI_I2C_DEVICE_ADDRESS, msg,
                       (count + 1), false);
    return num_bytes_read;
}

/**
 * Thin wrapper around reading from the I2C bus
 */
static int8_t imu_read_i2c(uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    // LOG_INFO("Reading %d bytes from address 0x%x", count, reg_addr);

    int num_bytes_read = 0;
    // Check to make sure caller is asking for 1 or more bytes
    if (count < 1)
    {
        return 0;
    }
    // Read data from register(s) over I2C
    i2c_write_blocking(SAMWISE_ADCS_IMU_I2C, BMI_I2C_DEVICE_ADDRESS, &reg_addr,
                       1, true);
    num_bytes_read = i2c_read_blocking(
        SAMWISE_ADCS_IMU_I2C, BMI_I2C_DEVICE_ADDRESS, reg_data, count, false);
    // LOG_INFO("DATA:");
    // for (int i = 0; i < count; i++)
    // {
    //     printf("%x ", reg_data[i]);
    // }
    return num_bytes_read;
}

/*!                User interface functions                                   */

/*!
 * I2C read function map to ADCS platform
 */
BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                    uint32_t len, void *intf_ptr)
{

    return imu_read_i2c(reg_addr, reg_data, (uint16_t)len);
}

/*!
 * I2C write function map to ADCS platform
 */
BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                     uint32_t len, void *intf_ptr)
{
    return imu_write_i2c(reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}

/*!
 * Delay function map to ADCS platform
 */
void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    sleep_us(period);
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
static int8_t bmi2_interface_init(struct bmi2_dev *bmi, uint8_t intf)
{
    if (bmi == NULL)
    {
        return BMI2_E_NULL_PTR;
    }

    // Initialize bmi over I2C
    LOG_INFO("Initializing BMI270 over I2C...\n");

    /* To initialize the user I2C function */
    bmi->intf = BMI2_I2C_INTF;
    bmi->read = bmi2_i2c_read;
    bmi->write = bmi2_i2c_write;

    /* Assign device address and bus instance to interface pointer */
    intf_conf.bus = bus_inst;
    intf_conf.dev_addr = BMI_I2C_DEVICE_ADDRESS;
    bmi->intf_ptr = ((void *)&intf_conf);

    /* Configure delay in microseconds */
    bmi->delay_us = bmi2_delay_us;

    /* Configure max read/write length (in bytes) ( Supported length depends on
     * target machine) */
    bmi->read_write_len = READ_WRITE_LEN;

    /* Assign to NULL to load the default config file. */
    bmi->config_file_ptr = NULL;

    return BMI2_OK;
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
static void bmi2_error_codes_print_result(int8_t rslt)
{
    switch (rslt)
    {
        case BMI2_OK:

            /* Do nothing */
            break;

        case BMI2_W_FIFO_EMPTY:
            printf("Warning [%d] : FIFO empty\r\n", rslt);
            break;
        case BMI2_W_PARTIAL_READ:
            printf("Warning [%d] : FIFO partial read\r\n", rslt);
            break;
        case BMI2_E_NULL_PTR:
            printf("Error [%d] : Null pointer error. It occurs when the user "
                   "tries to assign value (not address) to a pointer,"
                   " which has been initialized to NULL.\r\n",
                   rslt);
            break;

        case BMI2_E_COM_FAIL:
            printf("Error [%d] : Communication failure error. It occurs due to "
                   "read/write operation failure and also due "
                   "to power failure during communication\r\n",
                   rslt);
            break;

        case BMI2_E_DEV_NOT_FOUND:
            printf("Error [%d] : Device not found error. It occurs when the "
                   "device chip id is incorrectly read\r\n",
                   rslt);
            break;

        case BMI2_E_INVALID_SENSOR:
            printf("Error [%d] : Invalid sensor error. It occurs when there is "
                   "a mismatch in the requested feature with the "
                   "available one\r\n",
                   rslt);
            break;

        case BMI2_E_SELF_TEST_FAIL:
            printf("Error [%d] : Self-test failed error. It occurs when the "
                   "validation of accel self-test data is "
                   "not satisfied\r\n",
                   rslt);
            break;

        case BMI2_E_INVALID_INT_PIN:
            printf("Error [%d] : Invalid interrupt pin error. It occurs when "
                   "the user tries to configure interrupt pins "
                   "apart from INT1 and INT2\r\n",
                   rslt);
            break;

        case BMI2_E_OUT_OF_RANGE:
            printf("Error [%d] : Out of range error. It occurs when the data "
                   "exceeds from filtered or unfiltered data from "
                   "fifo and also when the range exceeds the maximum range for "
                   "accel and gyro while performing FOC\r\n",
                   rslt);
            break;

        case BMI2_E_ACC_INVALID_CFG:
            printf("Error [%d] : Invalid Accel configuration error. It occurs "
                   "when there is an error in accel configuration"
                   " register which could be one among range, BW or filter "
                   "performance in reg address 0x40\r\n",
                   rslt);
            break;

        case BMI2_E_GYRO_INVALID_CFG:
            printf("Error [%d] : Invalid Gyro configuration error. It occurs "
                   "when there is a error in gyro configuration"
                   "register which could be one among range, BW or filter "
                   "performance in reg address 0x42\r\n",
                   rslt);
            break;

        case BMI2_E_ACC_GYR_INVALID_CFG:
            printf("Error [%d] : Invalid Accel-Gyro configuration error. It "
                   "occurs when there is a error in accel and gyro"
                   " configuration registers which could be one among range, "
                   "BW or filter performance in reg address 0x40 "
                   "and 0x42\r\n",
                   rslt);
            break;

        case BMI2_E_CONFIG_LOAD:
            printf("Error [%d] : Configuration load error. It occurs when "
                   "failure observed while loading the configuration "
                   "into the sensor\r\n",
                   rslt);
            break;

        case BMI2_E_INVALID_PAGE:
            printf("Error [%d] : Invalid page error. It occurs due to failure "
                   "in writing the correct feature configuration "
                   "from selected page\r\n",
                   rslt);
            break;

        case BMI2_E_SET_APS_FAIL:
            printf("Error [%d] : APS failure error. It occurs due to failure "
                   "in write of advance power mode configuration "
                   "register\r\n",
                   rslt);
            break;

        case BMI2_E_AUX_INVALID_CFG:
            printf("Error [%d] : Invalid AUX configuration error. It occurs "
                   "when the auxiliary interface settings are not "
                   "enabled properly\r\n",
                   rslt);
            break;

        case BMI2_E_AUX_BUSY:
            printf("Error [%d] : AUX busy error. It occurs when the auxiliary "
                   "interface buses are engaged while configuring"
                   " the AUX\r\n",
                   rslt);
            break;

        case BMI2_E_REMAP_ERROR:
            printf("Error [%d] : Remap error. It occurs due to failure in "
                   "assigning the remap axes data for all the axes "
                   "after change in axis position\r\n",
                   rslt);
            break;

        case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
            printf("Error [%d] : Gyro user gain update fail error. It occurs "
                   "when the reading of user gain update status "
                   "fails\r\n",
                   rslt);
            break;

        case BMI2_E_SELF_TEST_NOT_DONE:
            printf("Error [%d] : Self-test not done error. It occurs when the "
                   "self-test process is ongoing or not "
                   "completed\r\n",
                   rslt);
            break;

        case BMI2_E_INVALID_INPUT:
            printf("Error [%d] : Invalid input error. It occurs when the "
                   "sensor input validity fails\r\n",
                   rslt);
            break;

        case BMI2_E_INVALID_STATUS:
            printf("Error [%d] : Invalid status error. It occurs when the "
                   "feature/sensor validity fails\r\n",
                   rslt);
            break;

        case BMI2_E_CRT_ERROR:
            printf("Error [%d] : CRT error. It occurs when the CRT test has "
                   "failed\r\n",
                   rslt);
            break;

        case BMI2_E_ST_ALREADY_RUNNING:
            printf("Error [%d] : Self-test already running error. It occurs "
                   "when the self-test is already running and "
                   "another has been initiated\r\n",
                   rslt);
            break;

        case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
            printf("Error [%d] : CRT ready for download fail abort error. It "
                   "occurs when download in CRT fails due to wrong "
                   "address location\r\n",
                   rslt);
            break;

        case BMI2_E_DL_ERROR:
            printf("Error [%d] : Download error. It occurs when write length "
                   "exceeds that of the maximum burst length\r\n",
                   rslt);
            break;

        case BMI2_E_PRECON_ERROR:
            printf("Error [%d] : Pre-conditional error. It occurs when "
                   "precondition to start the feature was not "
                   "completed\r\n",
                   rslt);
            break;

        case BMI2_E_ABORT_ERROR:
            printf("Error [%d] : Abort error. It occurs when the device was "
                   "shaken during CRT test\r\n",
                   rslt);
            break;

        case BMI2_E_WRITE_CYCLE_ONGOING:
            printf("Error [%d] : Write cycle ongoing error. It occurs when the "
                   "write cycle is already running and another "
                   "has been initiated\r\n",
                   rslt);
            break;

        case BMI2_E_ST_NOT_RUNING:
            printf("Error [%d] : Self-test is not running error. It occurs "
                   "when self-test running is disabled while it's "
                   "running\r\n",
                   rslt);
            break;

        case BMI2_E_DATA_RDY_INT_FAILED:
            printf("Error [%d] : Data ready interrupt error. It occurs when "
                   "the sample count exceeds the FOC sample limit "
                   "and data ready status is not updated\r\n",
                   rslt);
            break;

        case BMI2_E_INVALID_FOC_POSITION:
            printf("Error [%d] : Invalid FOC position error. It occurs when "
                   "average FOC data is obtained for the wrong"
                   " axes\r\n",
                   rslt);
            break;

        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}

/*!
 *  @brief This internal API is used to set configurations for gyro.
 */
static int8_t set_gyro_config(struct bmi2_dev *dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi2_sens_config config;

    /* Configure the type of feature. */
    config.type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(&config, 1, dev);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT2, dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* The user can change the following configuration parameters according
         * to their requirement. */
        /* Set Output Data Rate */
        config.cfg.gyr.odr = BMI2_GYR_ODR_50HZ;

        /* Gyroscope Angular Rate Measurement Range. Use the smallest for
         * highest accuracy. */
        config.cfg.gyr.range = BMI2_GYR_RANGE_125;

        /* Gyroscope bandwidth parameters. Use most agressive LPF. */
        config.cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate
         * sensing There are two modes 0 -> Ultra low power mode(Default) 1 ->
         * High performance mode
         */
        config.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the gyro configurations. */
        rslt = bmi2_set_sensor_config(&config, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    constexpr float power = 2.0f;
    const float half_scale =
        (float)((powf((float)power, (float)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

/*!
 * @brief This function converts lsb to radians per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_rps(int16_t val, float dps, uint8_t bit_width)
{
    return lsb_to_dps(val, dps, bit_width) * RAD_TO_DEG;
}

// *******************************

/**
 * Turn on power to the IMU.
 */
void imu_power_enable()
{
    gpio_put(SAMWISE_ADCS_EN_IMU, 0);
}

/**
 * Turn off power to the IMU.
 */
void imu_power_disable()
{
    gpio_put(SAMWISE_ADCS_EN_IMU, 1);
}

static void init_imu_pins()
{
    // Set power enable to output
    gpio_init(SAMWISE_ADCS_EN_IMU);
    gpio_set_dir(SAMWISE_ADCS_EN_IMU, GPIO_OUT);
    gpio_put(SAMWISE_ADCS_EN_IMU, 0);

    // Set IMU pins as inputs
    gpio_init(SAMWISE_ADCS_IMU_INT1);
    gpio_init(SAMWISE_ADCS_IMU_INT2);
    gpio_set_dir(SAMWISE_ADCS_IMU_INT1, GPIO_IN);
    gpio_set_dir(SAMWISE_ADCS_IMU_INT2, GPIO_IN);
}

/**
 * Initialize the IMU.
 */
bool imu_init()
{
    // Initialize pins and turn on power to the IMU
    init_imu_pins();
    imu_power_enable();
    sleep_ms(10);

    /* Assign gyro sensor to variable. */
    uint8_t sens_list = BMI2_GYRO;
    int8_t result;

    result = bmi2_interface_init(&bmi, BMI2_I2C_INTF);
    bmi2_error_codes_print_result(result);
    if (result != BMI2_OK)
    {
        return false;
    }

    /* Perform soft reset */
    bmi2_soft_reset(&bmi);
    sleep_ms(10);

    /* Initialize bmi270_legacy. */
    result = bmi270_legacy_init(&bmi);
    bmi2_error_codes_print_result(result);
    if (result != BMI2_OK)
    {
        return false;
    }
    sleep_ms(150);

    /* Turn off advanced power saving */
    result = bmi2_set_adv_power_save(BMI2_DISABLE, &bmi);
    bmi2_error_codes_print_result(result);
    if (result != BMI2_OK)
    {
        return false;
    }
    sleep_us(500);

    result = set_gyro_config(&bmi);
    bmi2_error_codes_print_result(result);
    if (result != BMI2_OK)
    {
        return false;
    }

    /* Verify chip ID */
    uint8_t chip_id = 0;
    bmi2_get_regs(0x00, &chip_id, 1, &bmi);

    LOG_INFO("IMU chip ID is %x", chip_id);
    if (chip_id != BMI_EXPECTED_CHIP_ID)
    {
        return false;
    }

    /* Enable the selected sensors. */
    result = bmi2_sensor_enable(&sens_list, 1, &bmi);
    bmi2_error_codes_print_result(result);
    if (result != BMI2_OK)
    {
        return false;
    }
    sleep_ms(3);

    return true;
}

/**
 * @brief Gets the current rotation rate from the IMU
 *
 * @param w_out     Pointer to put angular speed (in radians per second)
 * @return true if data is available, false otherwise
 */
bool imu_get_rotation(float3 *w_out)
{
    /* Structure to define type of sensor and their respective data. */
    struct bmi2_sens_data sensor_data;

    int8_t result = bmi2_get_sensor_data(&sensor_data, &bmi);

    uint8_t state = gpio_get(SAMWISE_ADCS_IMU_INT1);

    if ((result == BMI2_OK) && (sensor_data.status & BMI2_DRDY_GYR))
    {
        /*
         * Converting lsb to radians per second for 16 bit gyro at 125 dps
         range.
         */
        const float x = lsb_to_rps(sensor_data.gyr.x, 125.0f, bmi.resolution);
        const float y = lsb_to_rps(sensor_data.gyr.y, 125.0f, bmi.resolution);
        const float z = lsb_to_rps(sensor_data.gyr.z, 125.0f, bmi.resolution);

        /*
         * Apply calibration offset
         */
        float3 w_raw(x, y, z);
        *w_out = w_raw - IMU_ZERO_READING_RPS;

        return true;
    }

    LOG_DEBUG("Reading the gyro with no data!");
    return false;
}