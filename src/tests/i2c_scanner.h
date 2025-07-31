/**
 * @author  Lundeen Cahilly and Yiheng
 * @date    2025-07-20
 *
 * i2c scanner task from picubed flight software
 */

#pragma once

#include <stdbool.h>

/*
 * Read the I2C bus for devices
 *
 * This function scans the I2C bus for devices by attempting to read from each
 * valid 7-bit I2C address. It logs the addresses of any devices that respond.
 * If no devices are found, it logs an error message.
 *
 * @return void
 */
void i2c_scanner(void);