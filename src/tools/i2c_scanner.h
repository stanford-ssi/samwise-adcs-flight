/**
 * @author  Lundeen Cahilly and Yiheng
 * @date    2025-07-20
 *
 * i2c scanner task from picubed flight software
 */

#pragma once

#include "hardware/i2c.h"
#include <stdbool.h>

/*
 * Read the I2C bus for devices
 *
 * @param i2c_inst The I2C instance to scan
 * @param bus_name Name for logging (e.g., "I2C0", "I2C1")
 * @return void
 */
void scan_i2c_bus(i2c_inst_t *i2c_inst, const char *bus_name);