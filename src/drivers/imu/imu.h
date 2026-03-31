/**
 * @author Niklas Vainio
 * @date 2025-05-08
 *
 * This file defines utilities for controlling the BMI270 IMU
 *
 * This driver was entirely made possibly by the tireless work of Pete Mahowland
 */
#include "linalg.h"
using namespace linalg::aliases;

#pragma once

typedef struct IMU_data {
    bool imu_alive;
    bool imu_data_valid;
    float3 w_body_raw; // [rad/s] in body frame, raw reading
    float3 w_body;     // [rad/s] in body frame, low-pass filtered
    float w_mag;       // [rad/s] overall magnitude in body frame
    float3 a_body; // [km/s^2] specific force in body frame (non-gravitational
                   // acceleration)
} imu_data_t;

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
bool imu_init();
bool imu_get_rotation(float3 *w_out);
bool imu_get_accel(float3 *a_out);
