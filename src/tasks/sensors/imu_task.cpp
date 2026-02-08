/**
 * @author Lundeen Cahilly, Niklas Vainio
 * @date 2025-10-18
 *
 * Fast IMU task for reading gyroscope data at 50 Hz.
 * This is the primary sensor for the attitude filter and control loops.
 */

#include "imu_task.h"
#include "params.h"
#include "macros.h"

#include "drivers/imu/imu.h"
#include "gnc/estimation/attitude_filter.h"
#include "gnc/estimation/orbit_filter.h"
#include "gnc/utils/utils.h"
#include "pico/time.h"
#include <cmath>

/**
 * @brief Initialize IMU task.
 *
 * @param slate Pointer to the current satellite slate
 */
void imu_task_init(slate_t *slate)
{
    LOG_INFO("[sensor] Initializing IMU...");
    bool imu_result = imu_init();
    slate->imu_alive = imu_result;

    if (!imu_result)
    {
        LOG_ERROR("[sensor] Error initializing IMU - deactivating!");
    }

    slate->imu_data_valid = false;

    LOG_INFO("[sensor] IMU Initialization Complete! IMU alive: %s",
             slate->imu_alive ? "true" : "false");
}

/**
 * @brief Dispatch IMU task. Reads gyroscope at 50 Hz.
 *
 * @param slate Pointer to the current satellite slate
 */
void imu_task_dispatch(slate_t *slate)
{
    if (!slate->imu_alive)
    {
        LOG_DEBUG("[sensor] Skipping IMU due to invalid initialization!");
        return;
    }

    // Read gyro data
    bool gyro_result = imu_get_rotation(&slate->w_body_raw);

    if (gyro_result)
    {
        // Apply low pass filter - we run at 50 Hz so alpha = 2*pi*fc*dt
        // For fc = 1 Hz cutoff, dt = 0.02s (50 Hz): alpha = 2*pi*1*0.02 =
        // 0.1257
        constexpr float imu_lpf_alpha = 0.125663706144;
        slate->w_body =
            low_pass_filter(slate->w_body, slate->w_body_raw, imu_lpf_alpha);

        // Update magnitude
        slate->w_mag = length(slate->w_body);

        if (slate->af_is_initialized)
        {
            attitude_filter_propagate(slate);
        }

        LOG_DEBUG("[sensor] w_body = [%.5f, %.5f, %.5f]", slate->w_body[0],
                  slate->w_body[1], slate->w_body[2]);

        // Propagate attitude filter with gyro data
        if (slate->af_is_initialized)
        {
            attitude_filter_propagate(slate);
        }
    }

    // Read accelerometer data separately
    bool accel_result = imu_get_accel(&slate->a_body);

    if (accel_result)
    {
        LOG_DEBUG("[sensor] a_body = [%.6f, %.6f, %.6f] km/s^2",
                  slate->a_body.x, slate->a_body.y, slate->a_body.z);

        // Propagate orbit filter with accel data (requires quaternion from
        // attitude filter)
        if (slate->of_is_initialized)
        {
            orbit_filter_propagate(slate);
        }
    }

    slate->imu_data_valid = gyro_result;
}

sched_task_t imu_task = {.name = "imu",
                         .dispatch_period_ms = 20, // 50 Hz
                         .task_init = &imu_task_init,
                         .task_dispatch = &imu_task_dispatch,
                         .next_dispatch = 0};
