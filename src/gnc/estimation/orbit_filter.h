/**
 * @author Lundeen Cahilly
 * @date 2025-10-25
 *
 * This file implements an orbit Kalman filter using RK4 integration
 * for position and velocity estimation from GPS measurements
 */

#pragma once

#include "linalg.h"

using namespace linalg::aliases;

#include "drivers/gps/gps.h"
#include "drivers/imu/imu.h"
#include "gnc/mekf/filter.h"
#include "pico/time.h"

struct orbit_filter_t{
    float P_orbit[6 * 6];
    float P_orbit_log_frobenius;
    bool of_is_initialized = true;
    uint32_t of_init_count = 0;
    absolute_time_t of_last_propagate_time = get_absolute_time();
};

void orbit_filter_init(gps_data_processed_t &gps,
        imu_data_t &imu,
        orbit_filter_t &orbit_filter);

void orbit_filter_propagate(gps_data_processed_t &gps,
        imu_data_t &imu,
        orbit_filter_t &orbit_filter,
        AttitudeFilter attitude);

void orbit_filter_update(gps_data_processed_t &gps,
        imu_data_t &imu,
        orbit_filter_t &orbit_filter);

#ifdef TEST
void orbit_filter_stationary_test(gps_data_processed_t &gps,
        imu_data_t &imu,
        orbit_filter_t &orbit_filter,
        AttitudeFilter attitude);

void orbit_filter_convergence_test(gps_data_processed_t &gps,
        imu_data_t &imu,
        orbit_filter_t &orbit_filter,
        AttitudeFilter attitude);

void orbit_filter_multi_orbit_test(gps_data_processed_t &gps,
        imu_data_t &imu,
        orbit_filter_t &orbit_filter,
        AttitudeFilter attitude);

void orbit_filter_multi_orbit_gps_test(gps_data_processed_t &gps,
        imu_data_t &imu,
        orbit_filter_t &orbit_filter,
        AttitudeFilter attitude);
#endif
