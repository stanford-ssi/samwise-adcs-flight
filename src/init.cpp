/**
 * @file init.h
 * @author Niklas Vainio
 * @date 2025-05-08
 *
 * This file defines the main function for initializing hardware on the board
 */

#include "init.h"
#include "macros.h"
#include "pins.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include "drivers/imu.h"

/**
 * @brief Initialize pins for IMU busses
 *
 */
static void init_i2c_buses()
{
    // Initialize I2c pins and buses
    gpio_init(SAMWISE_ADCS_I2C0_SCL);
    gpio_init(SAMWISE_ADCS_I2C0_SDA);
    gpio_set_function(SAMWISE_ADCS_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_set_function(SAMWISE_ADCS_I2C0_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SAMWISE_ADCS_I2C0_SCL);
    gpio_pull_up(SAMWISE_ADCS_I2C0_SDA);

    gpio_init(SAMWISE_ADCS_I2C1_SCL);
    gpio_init(SAMWISE_ADCS_I2C1_SDA);
    gpio_set_function(SAMWISE_ADCS_I2C1_SCL, GPIO_FUNC_I2C);
    gpio_set_function(SAMWISE_ADCS_I2C1_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SAMWISE_ADCS_I2C1_SCL);
    gpio_pull_up(SAMWISE_ADCS_I2C1_SDA);

    i2c_init(i2c0, SAMWISE_ADCS_I2C0_BAUD);
    i2c_init(i2c1, SAMWISE_ADCS_I2C1_BAUD);
}

// **************************

void init(slate_t *slate)
{
    LOG_INFO("Intializing...");

    init_i2c_buses();
}