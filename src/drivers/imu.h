/**
 * @author Niklas Vainio
 * @date 2025-05-08
 *
 * This file defines utilities for controlling the BMI270 IMU
 *
 * This driver was entirely made possibly by the tireless work of Pete Mahowland
 */

#include "slate.h"

#pragma once

/*!
 * @brief  Structure to store the interface related configurations
 */
struct adcs_intf_config
{
    uint8_t
        dev_addr; /* Device address or Chip select of the interface selected */
    uint8_t bus;  /* Bus instance of the interface selected */
};

void imu_power_disable();
void imu_power_enable();
void imu_init(slate_t *slate);
void imu_get_rotation(slate_t *slate);
