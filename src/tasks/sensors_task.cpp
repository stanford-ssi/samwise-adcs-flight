/**
 * @author Niklas Vainio
 * @date 2025-06-02
 *
 * This task is responsible for reasding data from sensors (magmeter, IMU, GPS,
 * sun sensors) and putting in in the slate. It runs in all states.
 *
 */

#include "sensors_task.h"
#include "macros.h"

#include "drivers/imu.h"
#include "drivers/magnetometer.h"
#include "gnc/utils.h"
#include "pico/time.h"

void sensors_task_init(slate_t *slate)
{
    // Initialize all sensors
    LOG_INFO("[sensors] Initializing sensors...");

    // Magnetometers
    LOG_INFO("[sensors] Initializing magnetometer...");
    rm3100_error_t magmeter_result = rm3100_init();
    slate->magmeter_alive = (magmeter_result == RM3100_OK);

    if (magmeter_result != RM3100_OK)
    {
        LOG_ERROR("[sensors] Error initializing magnetometer - deactivating!");
    }

    // IMU
    LOG_INFO("[sensors] Initializing IMU...");
    bool imu_result = imu_init();
    slate->imu_alive = imu_result;

    if (!imu_result)
    {
        LOG_ERROR("[sensors] Error initializing IMU - deactivating!");
    }

    // Ensure all sensor data valid flags are false
    slate->magmeter_data_valid = false;
    slate->imu_data_valid = false;
    // slate->sun_sensor_data_valid = false;
    // slate->gps_data_valid = false;

    LOG_INFO("[sensors] Sensor Initialization Complete!");
}

void sensors_task_dispatch(slate_t *slate)
{
    // Read all sensors

    // Magnetometer
    if (slate->magmeter_alive)
    {
        LOG_DEBUG("[sensors] Reading magnetometer...");
        rm3100_error_t result = rm3100_get_reading(&slate->b_field_local);

        slate->magmeter_data_valid = (result == RM3100_OK);
        slate->b_field_read_time = get_absolute_time();

        slate->bdot_data_has_updated = true; // Set flag for bdot
    }
    else
    {
        LOG_DEBUG(
            "[sensors] Skipping magnetometer due to invalid initialization!");
    }

    // IMU
    if (slate->imu_alive)
    {
        LOG_DEBUG("[sensors] Reading IMU...");
        bool result = imu_get_rotation(&slate->w_body_raw);

        // Apply low pass filter - we run at 10 Hz so this achieves a cutoff of
        // roughly 1Hz
        if (result)
        {
            constexpr float imu_lpf_alpha = 0.628318530718;
            slate->w_body_filtered = low_pass_filter(
                slate->w_body_filtered, slate->w_body_raw, imu_lpf_alpha);

            // Update magnitude
            slate->w_mag = length(slate->w_body_filtered);
        }

        slate->imu_data_valid = result;
    }
    else
    {
        LOG_DEBUG("[sensors] Skipping IMU due to invalid initialization!");
    }
}

sched_task_t sensors_task = {.name = "sensors",
                             .dispatch_period_ms = 100,
                             .task_init = &sensors_task_init,
                             .task_dispatch = &sensors_task_dispatch,

                             /* Set to an actual value on init */
                             .next_dispatch = 0};
