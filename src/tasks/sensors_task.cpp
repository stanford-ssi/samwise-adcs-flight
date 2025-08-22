/**
 * @author Niklas Vainio
 * @date 2025-06-02
 *
 * This task is responsible for reading data from sensors (magmeter, IMU, GPS,
 * sun sensors) and putting in in the slate. It runs in all states.
 *
 */

#include "sensors_task.h"
#include "constants.h"
#include "macros.h"

#include "drivers/adm1176.h"
#include "drivers/gps.h"
#include "drivers/imu.h"
#include "drivers/magnetometer.h"
#include "drivers/magnetorquer.h"
#include "drivers/photodiodes_yz.h"
#include "drivers/sun_pyramids.h"
#include "gnc/utils.h"
#include "pico/time.h"
#include <cmath>

void sensors_task_init(slate_t *slate)
{
    LOG_INFO("[sensors] Initializing sensors...");

    // --- ADCS Power Monitor --- //
    LOG_INFO("[sensors] Initializing ADCS Power Monitor...");
    bool adm1176_result = adm_init();
    slate->adm1176_alive = adm1176_result;

    if (!adm1176_result)
    {
        LOG_ERROR(
            "[sensors] Error initializing ADCS Power Monitor - deactivating!");
        adm_power_off();
    }

    // --- Magnetometer --- //
    LOG_INFO("[sensors] Initializing magnetometer...");
    rm3100_error_t magmeter_result = rm3100_init();
    slate->magmeter_alive = (magmeter_result == RM3100_OK);

    if (magmeter_result != RM3100_OK)
    {
        LOG_ERROR("[sensors] Error initializing magnetometer - deactivating!");
    }

    // --- GPS --- //
    LOG_INFO("[sensors] Initializing GPS...");
    bool gps_result = gps_init();
    slate->gps_alive = gps_result;

    if (!gps_result)
    {
        LOG_ERROR("[sensors] Error initializing GPS - deactivating!");
    }

    // --- Sun Sensors --- //
    LOG_INFO("[sensors] Initializing sun pyramids...");
    bool sun_pyramids_result = sun_pyramids_init();
    slate->sun_pyramids_alive = sun_pyramids_result;

    if (!sun_pyramids_result)
    {
        LOG_ERROR("[sensors] Error initializing sun pyramids - deactivating!");
    }

    LOG_INFO("[sensors] Initializing photodiodes_yz...");
    bool photodiodes_yz_result = photodiodes_yz_init();
    slate->photodiodes_yz_alive = photodiodes_yz_result;

    if (!photodiodes_yz_result)
    {
        LOG_ERROR(
            "[sensors] Error initializing photodiodes_yz - deactivating!");
    }

    // --- IMU --- //
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
    slate->sun_pyramids_data_valid = false;
    slate->gps_data_valid = false;

    LOG_INFO("[sensors] Sensor Initialization Complete! Magmeter alive: %s, "
             "IMU alive: %s",
             "Sun pyramids alive: %s, GPS alive: %s",
             slate->magmeter_alive ? "true" : "false",
             slate->imu_alive ? "true" : "false",
             slate->sun_pyramids_alive ? "true" : "false",
             slate->gps_alive ? "true" : "false");
}

void sensors_task_dispatch(slate_t *slate)
{
    // Read all sensors

    // --- ADCS Power Monitor --- //
    if (slate->adm1176_alive)
    {
        bool result = adm_get_power(slate);
        LOG_DEBUG("[sensors] ADCS Power Monitor [%.3fW, %.3fV, "
                  "%.3fA]",
                  slate->adcs_power, slate->adcs_voltage, slate->adcs_current);
    }
    else
    {
        LOG_DEBUG("[sensors] Skipping ADCS Power Monitor due to invalid "
                  "initialization!");
    }

    // --- Magnetometer --- //
    if (slate->magmeter_alive)
    {
        rm3100_error_t result;

        // If magnetorquers are running, we need to turn them off for some
        // settle time to read the magnetometer to avoid interference
        if (slate->magnetorquers_running)
        {
            // Toggle magnetorquers off temporarily
            stop_magnetorquer_pwm();
            slate->magnetorquers_running = false;

            // Wait for magnetometer field to settle
            sleep_ms(MAGNETOMETER_FIELD_SETTLE_TIME_MS);

            // Read magnetometer
            result = rm3100_get_reading(&slate->b_field_local);

            // Turn magnetorquers back on after reading
            bool mag_result = do_magnetorquer_pwm(slate->magdrv_requested);
            if (!mag_result)
            {
                LOG_ERROR("[sensors] Error reactivating magnetorquers");
            }
        }
        else
        {
            // Magnetorquers not running, read directly
            result = rm3100_get_reading(&slate->b_field_local);
        }
        // Calculate compass bearing and inclination
        // +X = North, +Y = West, -Z = Down
        float bearing_deg =
            atan2f(-slate->b_field_local.y, slate->b_field_local.x) * 180.0f /
            M_PI;
        if (bearing_deg < 0)
            bearing_deg += 360.0f;

        float inclination_deg =
            atan2f(-slate->b_field_local.z,
                   sqrtf(slate->b_field_local.x * slate->b_field_local.x +
                         slate->b_field_local.y * slate->b_field_local.y)) *
            180.0f / M_PI;

        LOG_DEBUG("[sensors] Magnetometer reading: [%.3f, %.3f, %.3f] | "
                  "Bearing: %.1f° | Inclination: %.1f°",
                  slate->b_field_local.x, slate->b_field_local.y,
                  slate->b_field_local.z, bearing_deg, inclination_deg);

        slate->magmeter_data_valid = (result == RM3100_OK);
        slate->b_field_read_time = get_absolute_time();

        slate->bdot_data_has_updated = true; // Set flag for bdot
    }
    else
    {
        LOG_DEBUG(
            "[sensors] Skipping magnetometer due to invalid initialization!");
    }

    // --- GPS --- //
    if (slate->gps_alive)
    {
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

    // --- IMU --- //
    if (slate->imu_alive)
    {
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

    // --- Sun Sensors --- //
    if (slate->sun_pyramids_alive)
    {
        uint8_t adc_values[8];
        float voltages[8];
        bool result = sun_pyramids_read_all_channels(adc_values);
        bool voltage_result = sun_pyramids_read_all_voltages(voltages);

        if (result)
        {
            for (int i = 0; i < 8; i++)
            {
                // Scale to 12-bit range (0-4095) for consistency
                uint16_t normalized_intensity =
                    static_cast<uint16_t>(adc_values[i]) *
                    SUN_SENSOR_CLIP_VALUE / MAX_VALUE_ADS7830;
                slate->sun_sensors_intensities[i] = normalized_intensity;
            }

            // Set remaining sun sensor values to 0 if not connected
            for (int i = 8; i < NUM_SUN_SENSORS; i++)
            {
                slate->sun_sensors_intensities[i] = 0.0f;
            }
        }

        if (voltage_result)
        {
            // Store voltage readings for sun pyramids (channels 0-7)
            for (int i = 0; i < 8; i++)
            {
                slate->sun_sensors_voltages[i] = voltages[i];
            }
        }
        slate->sun_pyramids_data_valid = result;
    }
    else
    {
        LOG_DEBUG(
            "[sensors] Skipping sun pyramids due to invalid initialization!");
    }

    if (slate->photodiodes_yz_alive)
    {
        uint16_t adc_values[8];
        float voltages[8];
        bool result = photodiodes_yz_read_all_channels(adc_values);
        bool voltage_result = photodiodes_yz_read_all_voltages(voltages);

        if (result)
        {
            for (int i = 0; i < 8; i++)
            {
                // Clip to 2.5V max value for consistency
                uint16_t normalized_intensity =
                    min(adc_values[i], SUN_SENSOR_CLIP_VALUE);
                slate->sun_sensors_intensities[i + 8] = normalized_intensity;
            }
        }

        if (voltage_result)
        {
            // Store voltage readings for YZ sensors (channels 8-15)
            for (int i = 0; i < 8; i++)
            {
                slate->sun_sensors_voltages[i + 8] = voltages[i];
            }
        }
        slate->photodiodes_yz_data_valid = result;
    }

    // Log all sun sensor readings after collecting all data
    // LOG_DEBUG(
    //     "[sensors] Sun sensor readings: [%u, %u, %u, %u, "
    //     "%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, "
    //     "%u, %u]",
    //     slate->sun_sensors_intensities[0], slate->sun_sensors_intensities[1],
    //     slate->sun_sensors_intensities[2], slate->sun_sensors_intensities[3],
    //     slate->sun_sensors_intensities[4], slate->sun_sensors_intensities[5],
    //     slate->sun_sensors_intensities[6], slate->sun_sensors_intensities[7],
    //     slate->sun_sensors_intensities[8], slate->sun_sensors_intensities[9],
    //     slate->sun_sensors_intensities[10],
    //     slate->sun_sensors_intensities[11],
    //     slate->sun_sensors_intensities[12],
    //     slate->sun_sensors_intensities[13],
    //     slate->sun_sensors_intensities[14],
    //     slate->sun_sensors_intensities[15]);

    LOG_DEBUG("[sensors] Sun sensor voltages: [%.2f, %.2f, %.2f, %.2f, "
              "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, "
              "%.2f, %.2f]",
              slate->sun_sensors_voltages[0], slate->sun_sensors_voltages[1],
              slate->sun_sensors_voltages[2], slate->sun_sensors_voltages[3],
              slate->sun_sensors_voltages[4], slate->sun_sensors_voltages[5],
              slate->sun_sensors_voltages[6], slate->sun_sensors_voltages[7],
              slate->sun_sensors_voltages[8], slate->sun_sensors_voltages[9],
              slate->sun_sensors_voltages[10], slate->sun_sensors_voltages[11],
              slate->sun_sensors_voltages[12], slate->sun_sensors_voltages[13],
              slate->sun_sensors_voltages[14], slate->sun_sensors_voltages[15]);
}

sched_task_t sensors_task = {.name = "sensors",
                             .dispatch_period_ms = 100,
                             .task_init = &sensors_task_init,
                             .task_dispatch = &sensors_task_dispatch,

                             /* Set to an actual value on init */
                             .next_dispatch = 0};
