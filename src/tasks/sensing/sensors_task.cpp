/**
 * @author Niklas Vainio
 * @date 2025-06-02
 *
 * This task is responsible for reading data from sensors (magetometer, IMU,
 * GPS, sun sensors) and putting in in the slate. It runs in all states.
 *
 */

#include "sensors_task.h"
#include "constants.h"
#include "macros.h"

#include "drivers/gps/gps.h"
#include "drivers/imu/imu.h"
#include "drivers/magnetometer/magnetometer.h"
#include "drivers/magnetorquers/magnetorquers.h"
#include "drivers/power_monitor/power_monitor.h"
#include "drivers/sun_sensors/ads7830.h"
#include "drivers/sun_sensors/rp2350b_adc.h"
#include "gnc/utils/mjd.h"
#include "gnc/utils/utils.h"
#include "pico/time.h"
#include <cmath>

/**
 * @brief Initialize sensors task. Initializes all sensors and sets alive
 * and data valid flags in the slate.
 *
 * @param slate Pointer to the current satellite slate
 */
void sensors_task_init(slate_t *slate)
{
    LOG_INFO("[sensors] Initializing sensors");

    // --- ADCS Power Monitor --- //
    LOG_INFO("[sensors] Initializing ADCS Power Monitor...");
    bool adm1176_result = adm_init();
    slate->power_monitor_alive = adm1176_result;

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
    LOG_INFO("[sensors] Initializing sun sensors rp2350b adc...");
    bool rp2350b_adc_result = rp2350b_adc_init();
    if (!rp2350b_adc_result)
    {
        LOG_ERROR("[sensors] Error initializing rp2350b_adc - deactivating!");
    }

    LOG_INFO("[sensors] Initializing sun sensors ads7830...");
    bool ads7830_result = ads7830_init();
    if (!ads7830_result)
    {
        LOG_ERROR("[sensors] Error initializing ads7830 - deactivating!");
    }

    // Write alive status for each sun sensor
    for (uint8_t i = 0; i < NUM_SUN_SENSORS / 2;
         i++) // first 8 are from rp2350b adc
    {
        if (i < NUM_SUN_SENSORS / 2)
        { // first 8 are from rp2350b adc
            slate->sun_sensor_alive[i] = rp2350b_adc_result;
        }
        else
        { // last 8 are from ads7830
            slate->sun_sensor_alive[i] = ads7830_result;
        }
        slate->sun_sensor_data_valid[i] =
            false; // initialize data valid to false
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
    slate->gps_data_valid = false;
    for (uint8_t i = 0; i < NUM_SUN_SENSORS; i++)
    {
        slate->sun_sensor_data_valid[i] = false;
    }
    slate->sun_vector_valid = false;

    LOG_INFO("[sensors] Sensor Initialization Complete! Magmeter alive: %s, "
             "IMU alive: %s",
             "Sun pyramids alive: %s, GPS alive: %s",
             slate->magmeter_alive ? "true" : "false",
             slate->imu_alive ? "true" : "false",
             slate->gps_alive ? "true" : "false");
    // log sun sensors alive by each but in one line
    LOG_INFO("[sensors] Sun sensors alive status:");
    for (uint8_t i = 0; i < NUM_SUN_SENSORS; i++)
    {
        LOG_INFO("  Sensor %2d: %s", i,
                 slate->sun_sensor_alive[i] ? "true" : "false");
    }
}

/**
 * @brief Dispatch sensors task. Reads all sensors and updates the slate
 * accordingly.
 *
 * @param slate Pointer to the current satellite slate
 */
void sensors_task_dispatch(slate_t *slate)
{
    // Read all sensors
    LOG_INFO("[sensors] Sensors task dispatching...");

    // --- ADCS Power Monitor --- //
    if (slate->power_monitor_alive)
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
            result = rm3100_get_reading(&slate->b_body);

            // Turn magnetorquers back on after reading
            bool mag_result = do_magnetorquer_pwm(slate->magnetorquer_moment);
            if (!mag_result)
            {
                LOG_ERROR("[sensors] Error reactivating magnetorquers");
            }
        }
        else
        {
            // Magnetorquers not running, read directly
            result = rm3100_get_reading(&slate->b_body);
        }
        LOG_DEBUG("[sensors] Magnetometer reading: [%.3f, %.3f, %.3f] | ",
                  slate->b_body.x, slate->b_body.y, slate->b_body.z);

        slate->magmeter_data_valid = (result == RM3100_OK);
        slate->b_body_read_time = get_absolute_time();

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
            slate->gps_alt = gps_data.altitude / 1000.0f; // Convert m to km
            slate->gps_time = static_cast<float>(
                gps_data.timestamp); // Convert HHMMSS to float

            // Parse GPS date (DDMMYY format) to UTC_date (year, month, day)
            uint32_t date = gps_data.date;
            uint32_t day = date / 10000;
            uint32_t month = (date / 100) % 100;
            uint32_t year = date % 100;

            // Convert 2-digit year to 4-digit (assume 21st century)
            year += 2000;

            slate->UTC_date[0] = static_cast<float>(year);  // Year
            slate->UTC_date[1] = static_cast<float>(month); // Month
            slate->UTC_date[2] = static_cast<float>(day);   // Day

            // Compute MJD based on GPS date and time
            compute_MJD(slate);

            LOG_DEBUG("[sensors] GPS data: Lat: %.6f, Lon: %.6f, Alt: %.3f, "
                      "Time: %.3f, "
                      "Date: %02d/%02d/%04d",
                      slate->gps_lat, slate->gps_lon, slate->gps_alt,
                      slate->gps_time, day, month, year);
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
            LOG_DEBUG("[sensors] IMU reading: [%.3f, %.3f, %.3f]",
                      slate->w_body_filtered[0], slate->w_body_filtered[1],
                      slate->w_body_filtered[2]);
        }
        LOG_DEBUG("[sensors] IMU reading: [%.7f, %.7f, %.7f]",
                  slate->w_body_raw.x, slate->w_body_raw.y,
                  slate->w_body_raw.z);

        slate->imu_data_valid = result;
    }
    else
    {
        LOG_DEBUG("[sensors] Skipping IMU due to invalid initialization!");
    }

    // --- Sun Sensors --- //
    bool rp2350b_adc_alive =
        slate->sun_sensor_alive[0]; // first 8 are from rp2350b adc
    bool ads7830_alive = slate->sun_sensor_alive[8]; // last 8 are from ads7830

    if (rp2350b_adc_alive)
    {
        uint16_t intensities[8];
        float voltages[8];
        bool result = rp2350b_adc_read_all_channels(intensities);
        bool voltage_result = rp2350b_adc_read_all_voltages(voltages);

        // Write intensities, voltages, and sensor status for channels 0-7 (sun
        // pyramids)
        int end = NUM_SUN_SENSORS / 2;
        for (int i = 0; i < end; i++)
        {
            if (result)
            {
                // Clip to 2.5V max value for consistency
                uint16_t normalized_intensity =
                    min(intensities[i], SUN_SENSOR_CLIP_VALUE);
                slate->sun_sensor_intensities[i] = normalized_intensity;
                slate->sun_sensor_data_valid[i] = true;
            }
            if (voltage_result)
            {
                slate->sun_sensor_voltages[i] = voltages[i];
            }
        }
    }
    else
    {
        LOG_DEBUG(
            "[sensors] Skipping rp2350b_adc due to invalid initialization!");
    }
    if (ads7830_alive)
    {
        uint8_t intensities[8];
        float voltages[8];
        bool result = ads7830_read_all_channels(intensities);
        bool voltage_result = ads7830_read_all_voltages(voltages);

        // Write intensities, voltages, and sensor status for channels 8-15 (YZ
        // sensors)
        int start = NUM_SUN_SENSORS / 2;
        for (int i = 0; i < start; i++)
        {
            if (result)
            {
                // Scale to 12-bit range (0-4095) for consistency
                uint16_t normalized_intensity =
                    static_cast<uint16_t>(intensities[i]) *
                    SUN_SENSOR_CLIP_VALUE / MAX_VALUE_ADS7830;
                slate->sun_sensor_intensities[i + start] = normalized_intensity;
                slate->sun_sensor_data_valid[i + start] = true;
            }
            if (voltage_result)
            {
                slate->sun_sensor_voltages[i + start] = voltages[i];
            }
        }
    }
    else
    {
        LOG_DEBUG("[sensors] Skipping ads7830 due to invalid initialization!");
    }

    LOG_DEBUG("[sensors] Sun sensor voltages: [%.2f, %.2f, %.2f, %.2f, "
              "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, "
              "%.2f, %.2f]",
              slate->sun_sensor_voltages[0], slate->sun_sensor_voltages[1],
              slate->sun_sensor_voltages[2], slate->sun_sensor_voltages[3],
              slate->sun_sensor_voltages[4], slate->sun_sensor_voltages[5],
              slate->sun_sensor_voltages[6], slate->sun_sensor_voltages[7],
              slate->sun_sensor_voltages[8], slate->sun_sensor_voltages[9],
              slate->sun_sensor_voltages[10], slate->sun_sensor_voltages[11],
              slate->sun_sensor_voltages[12], slate->sun_sensor_voltages[13],
              slate->sun_sensor_voltages[14], slate->sun_sensor_voltages[15]);
}

sched_task_t sensors_task = {.name = "sensors",
                             .dispatch_period_ms =
                                 100, // TODO: determine appropriate rate
                             .task_init = &sensors_task_init,
                             .task_dispatch = &sensors_task_dispatch,

                             /* Set to an actual value on init */
                             .next_dispatch = 0};
