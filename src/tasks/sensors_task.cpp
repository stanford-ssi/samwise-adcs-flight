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

#include "drivers/ads7830.h"
#include "drivers/gps.h"
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

    // GPS
    LOG_INFO("[sensors] Initializing GPS...");
    bool gps_result = gps_init();
    slate->gps_alive = gps_result;

    if (!gps_result)
    {
        LOG_ERROR("[sensors] Error initializing GPS - deactivating!");
    }

    // Sun Sensors
    LOG_INFO("[sensors] Initializing sun sensors...");
    bool sun_sensors_result = ads7830_init();
    slate->sun_sensors_alive = sun_sensors_result;

    if (!sun_sensors_result)
    {
        LOG_ERROR("[sensors] Error initializing sun sensors - deactivating!");
    }
    LOG_INFO("[sensors] Sun sensors initialized successfully!");

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
    slate->sun_sensors_data_valid = false;
    slate->gps_data_valid = false;

    LOG_INFO("[sensors] Sensor Initialization Complete! Magmeter alive: %s, "
             "IMU alive: %s",
             "Sun sensors alive: %s, GPS alive: %s",
             slate->magmeter_alive ? "true" : "false",
             slate->imu_alive ? "true" : "false",
             slate->sun_sensors_alive ? "true" : "false",
             slate->gps_alive ? "true" : "false");
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

    // GPS
    if (slate->gps_alive)
    {
        LOG_DEBUG("[sensors] Reading GPS...");
        gps_data_t gps_data;
        bool result =
            gps_get_data(&gps_data); // Only returns true if valid data AND fix

        if (result)
        {
            slate->gps_lat = gps_data.latitude;
            slate->gps_lon = gps_data.longitude;
            slate->gps_time =
                (float)gps_data.timestamp; // Convert HHMMSS to float
            LOG_INFO("[sensors] GPS data: Lat: %.6f, Lon: %.6f, Time: %.3f",
                     slate->gps_lat, slate->gps_lon, slate->gps_time);
        }

        slate->gps_data_valid = result;
    }
    else
    {
        LOG_DEBUG("[sensors] Skipping GPS due to invalid initialization!");
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

    // Sun Sensors
    if (slate->sun_sensors_alive)
    {
        LOG_DEBUG("[sensors] Reading sun sensors...");
        uint8_t adc_values[8];
        bool result = ads7830_read_all_channels(adc_values);

        if (result)
        {
            // Convert 8-bit ADC values to float intensities
            // ADC has 8 channels, but we have 10 sun sensors in the slate
            // Map the 8 ADC channels to the first 8 sun sensor positions
            for (int i = 0; i < 8; i++)
            {
                slate->sun_sensors_intensities[i] = (float)adc_values[i];
            }

            // Set remaining sun sensor values to 0 if not connected
            for (int i = 8; i < NUM_SUN_SENSORS; i++)
            {
                slate->sun_sensors_intensities[i] = 0.0f;
            }

            LOG_DEBUG("[sensors] Sun sensor readings: [%.1f, %.1f, %.1f, %.1f, "
                      "%.1f, %.1f, %.1f, %.1f]",
                      slate->sun_sensors_intensities[0],
                      slate->sun_sensors_intensities[1],
                      slate->sun_sensors_intensities[2],
                      slate->sun_sensors_intensities[3],
                      slate->sun_sensors_intensities[4],
                      slate->sun_sensors_intensities[5],
                      slate->sun_sensors_intensities[6],
                      slate->sun_sensors_intensities[7]);
        }

        slate->sun_sensors_data_valid = result;
    }
    else
    {
        LOG_DEBUG(
            "[sensors] Skipping sun sensors due to invalid initialization!");
    }
}

sched_task_t sensors_task = {.name = "sensors",
                             .dispatch_period_ms = 100,
                             .task_init = &sensors_task_init,
                             .task_dispatch = &sensors_task_dispatch,

                             /* Set to an actual value on init */
                             .next_dispatch = 0};
