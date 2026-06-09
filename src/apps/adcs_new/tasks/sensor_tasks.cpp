#include "FreeRTOS.h"
#include "task.h"

#include "linalg.h"
using namespace linalg;
using namespace linalg::aliases;

#include "pico/time.h"

#include "macros.h"

#include "apps/adcs_new/tasks/tasks.h"
#include "apps/adcs_new/params.h"
#include "apps/adcs_new/slate.h"

#include "drivers/telemetry/hitl.h"
#include "drivers/communications/protocol.h"
#include "drivers/sun_sensors/ads7830.h"
#include "drivers/sun_sensors/rp2350b_adc.h"
#include "drivers/sun_sensors/sun_sensors.h"

#include "gnc/estimation/sun_sensor_to_vector.h"
#include "gnc/utils/mjd.h"
#include "gnc/utils/utils.h"
#include "gnc/world/b_field.h"
#include "gnc/world/sun_vector.h"

extern slate_t slate;

#define GPS_SPOOFING 1

// This pretty much needs to be written anew
void vTaskGPS(void *) {
    stdio_flush();
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_GPS))
            TASK_LOOP_MS(200)
            LOG_INFO("GPS TASK");
#if GPS_SPOOFING
        slate.gps_data.gps_lat = 37.424732f;
        slate.gps_data.gps_lon = -122.180336f;
        slate.gps_data.gps_alt = 0.060f;
        slate.gps_data.gps_time = 6;
        slate.gps_data.MJD = 61199.767839;
        slate.gps_data.gps_data_valid = true;
        // ======================================================
        // UPDATE REFERENCE VECTORS
        // ======================================================
        compute_B(slate.gps_data, slate.b_field);
        compute_sun_vector_eci(slate.gps_data, slate.sun_vector);

        // ======================================================
        // SWITCH TO FUSION STATE
        // ======================================================
        if (slate.state_machine.current_state != STATE_FUSION) {
            StateMsg_t msg = MSG_GPS_VALID;
            xQueueSend(slate.state_machine.state_queue_handle,
                    &msg, 0);
        }
#else
        if (!slate.gps_data.gps_alive)
        {
            LOG_DEBUG("[sensor] Skipping GPS due to invalid initialization!");
            return;
        }

        gps_data_t gps_data;
        bool data_received =
            gps_get_data(&gps_data); // Only returns true if valid data AND fix

        if (data_received)
        {
            slate.gps_data.gps_read_time = get_absolute_time();

            slate.gps_data.gps_lat = gps_data.latitude;
            slate.gps_data.gps_lon = gps_data.longitude;
            slate.gps_data.gps_alt = 
                gps_data.altitude / 1000.0f; // Convert m to km
            slate.gps_data.gps_time = 183510.01f;
                static_cast<float>(gps_data.timestamp); // Convert HHMMSS to float
            slate.gps_data.gps_speed = gps_data.speed;          // knots
            slate.gps_data.gps_course = gps_data.course;        // degrees true

            // Parse GPS date (DDMMYY format) to UTC_date (year, month, day)
            uint32_t date = gps_data.date;
            uint32_t day = date / 10000;
            uint32_t month = (date / 100) % 100;
            uint32_t year = date % 100;

            // Convert 2-digit year to 4-digit (assume 21st century)
            year += 2000;

            slate.gps_data.UTC_date[0] = static_cast<float>(year);  // Year
            slate.gps_data.UTC_date[1] = static_cast<float>(month); // Month
            slate.gps_data.UTC_date[2] = static_cast<float>(day);   // Day

            // process UTC timestamp HHMMSS into UTC time in seconds
            uint32_t hhmmss = gps_data.timestamp;
            uint32_t h = hhmmss / 10000;
            uint32_t m = (hhmmss / 100) % 100;
            uint32_t s = hhmmss % 100;

            slate.gps_data.UTC_time = static_cast<float>(h * 3600 + m * 60 + s);
            // Compute MJD from GPS time/date
            slate.gps_data.MJD = 
                compute_MJD(slate.gps_data.UTC_date, 
                        slate.gps_data.gps_time);

            LOG_INFO("[GPS] Lat = %.6f, Lon = %.6f, Alt = %.3f, "
                      "Time = %.3f, "
                      "Time = %02d:%02d:%04d, "
                      "MJD = %.5f",
                      slate.gps_data.gps_lat, 
                      slate.gps_data.gps_lon, 
                      slate.gps_data.gps_alt,
                      slate.gps_data.gps_time,
                      h, m, s, slate.gps_data.MJD);

            slate.gps_data.gps_data_valid = true;

            // ======================================================
            // UPDATE REFERENCE VECTORS
            // ======================================================
            compute_B(slate.gps_data, slate.b_field);
            compute_sun_vector_eci(slate.gps_data, slate.sun_vector);
            // ======================================================
            // SWITCH TO FUSION STATE
            // ======================================================
            if (slate.state_machine.current_state != STATE_FUSION) {
                StateMsg_t msg = MSG_GPS_VALID;
                xQueueSend(slate.state_machine.state_queue_handle,
                        &msg, 0);
            }

        }

        // GPS data stays valid within a time frame, but becomes "stale" outside of
        // it
        else if (slate.gps_data.gps_data_valid &&
                 (get_absolute_time() - slate.gps_data.gps_read_time >
                  GPS_DATA_EXPIRATION_MS * 1000))    {
            slate.gps_data.gps_data_valid = false;
        }
#endif
    } // end for(;;)
} // end vTaskGPS

void vTaskIMU(void *) {
    stdio_flush();

    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_IMU))
        TASK_LOOP_MS(100)

        if (!slate.imu_data.imu_alive)
        {
            LOG_DEBUG("[sensor] Skipping IMU due to invalid initialization!");
            return;
        }

        // Read gyro data
        bool gyro_result = imu_get_rotation(&slate.imu_data.w_body_raw);

        if (gyro_result)
        {
            // Apply low pass filter - we run at 50 Hz so alpha = 2*pi*fc*dt
            // For fc = 1 Hz cutoff, dt = 0.02s (50 Hz): alpha = 2*pi*1*0.02 =
            // 0.1257
            constexpr float imu_lpf_alpha = 0.125663706144;
            slate.imu_data.w_body =
                low_pass_filter(slate.imu_data.w_body, 
                        slate.imu_data.w_body_raw,
                        imu_lpf_alpha);

            // Update magnitude
            slate.imu_data.w_mag = length(slate.imu_data.w_body);

            LOG_DEBUG("[IMU/GYRO] w_body = [%.5f, %.5f, %.5f]", 
                    slate.imu_data.w_body[0],
                    slate.imu_data.w_body[1],
                    slate.imu_data.w_body[2]);
        }

        // Read accelerometer data separately
        bool accel_result = imu_get_accel(&slate.imu_data.a_body);

        if (accel_result) {
            slate.imu_data.imu_accel_valid = true;

            LOG_DEBUG("[IMU/GYRO] a_body = [%.6f, %.6f, %.6f] km/s^2",
                    slate.imu_data.a_body.x,
                    slate.imu_data.a_body.y,
                    slate.imu_data.a_body.z);
        }

        slate.imu_data.imu_data_valid = gyro_result;

    } // end for(;;)
} // end vTaskIMU

void vTaskMagnetometer(void *) {
    // Track last read cycle start for 100ms period
    absolute_time_t last_mag_read_start = {0}; 

    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_MAGNETOMETER))
        TASK_LOOP_MS(20)

        rm3100_error_t result =
            rm3100_get_reading(&slate.magnetometer_data.b_body, 
                    &slate.magnetometer_data.b_body_raw);
        // Send data over usb
        msg_t mag_raw_msg;
        float data[3] = {slate.magnetometer_data.b_body_raw.x,
            slate.magnetometer_data.b_body_raw.y, 
            slate.magnetometer_data.b_body_raw.z};

        protocol_message_float3(&mag_raw_msg, (float*)&data);
        // send_hitl(&mag_raw_msg, 19);

        // TODO: Setup better calibration so that b_body is better than raw
        slate.magnetometer_data.b_body = slate.magnetometer_data.b_body_raw;

        slate.magnetometer_data.magnetometer_data_valid = (result == RM3100_OK);
        slate.magnetometer_data.b_body_read_time = get_absolute_time();
        last_mag_read_start = get_absolute_time();

        if (result != RM3100_OK) {
            LOG_INFO("[MAGNETOMETER] Error reading magnetometer");
        }
        // Update attitude filter with magnetometer measurement
        if (result == RM3100_OK) {
            LOG_INFO("[MAGNETOMETER] b_body = [%.3f, %.3f, %.3f]",
                    slate.magnetometer_data.b_body_raw.x,
                    slate.magnetometer_data.b_body_raw.y,
                    slate.magnetometer_data.b_body_raw.z);
        }
    } // end for(;;)
} // end vTaskMagnetometer

void vTaskSunSensor(void *) {
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_SUN_SENSOR))
        TASK_LOOP_MS(100)

        bool rp2350b_adc_alive = slate.sun_sensor.sun_sensor_alive[0];
        if (rp2350b_adc_alive)
        {
            uint16_t intensities[8];
            float voltages[8];
            bool result = rp2350b_adc_read_all_channels(intensities);
            bool voltage_result = rp2350b_adc_read_all_voltages(voltages);

            for (int i = 0; i < 8; i++)
            {
                if (result)
                {
                    // Clip to 2.5V max value for consistency with ads7830
                    uint16_t normalized_intensity =
                        min(intensities[i], SUN_SENSOR_SATURATION_VALUE);
                    slate.sun_sensor.intensities[i] = normalized_intensity;
                    slate.sun_sensor.data_valid[i] = true;
                }
                if (voltage_result)
                {
                    slate.sun_sensor.voltages[i] = voltages[i];
                }
            }
        }
        else
        {
            LOG_DEBUG(
                "[sensor] Skipping rp2350b_adc due to invalid initialization!");
        }

        // --- Read ads7830 ADC (sensors 8-15: Y/Z photodiodes) --- //
        bool ads7830_alive = slate.sun_sensor.sun_sensor_alive[8];
        if (ads7830_alive)
        {
            uint8_t intensities[8];
            float voltages[8];
            bool result = ads7830_read_all_channels(intensities);
            bool voltage_result = ads7830_read_all_voltages(voltages);

            for (int i = 0; i < 8; i++)
            {
                if (result)
                {
                    // Scale 8-bit to 12-bit range for consistency
                    uint16_t normalized_intensity =
                        static_cast<uint16_t>(intensities[i]) *
                        SUN_SENSOR_SATURATION_VALUE / MAX_VALUE_ADS7830;
                    slate.sun_sensor.intensities[i + 8] = normalized_intensity;
                    slate.sun_sensor.data_valid[i + 8] = true;
                }
                if (voltage_result)
                {
                    slate.sun_sensor.voltages[i + 8] = voltages[i];
                }
            }
        }
        else
        {
            LOG_DEBUG("[sun sensor] Skipping ads7830 due to invalid initialization!");
        }

        // Log all sun sensor intensities in an array format (uint32_t)
        // LOG_DEBUG("[sensor] Sun sensor intensities: ["
        //             "%.0u, %.0u, %.0u, %.0u, %.0u, %.0u, %.0u, %.0u, "
        //             "%.0u, %.0u, %.0u, %.0u, %.0u, %.0u, %.0u, %.0u]",
        //             slate.sun_sensor.intensities[0],
        //             slate.sun_sensor.intensities[1],
        //             slate.sun_sensor.intensities[2],
        //             slate.sun_sensor.intensities[3],
        //             slate.sun_sensor.intensities[4],
        //             slate.sun_sensor.intensities[5],
        //             slate.sun_sensor.intensities[6],
        //             slate.sun_sensor.intensities[7],
        //             slate.sun_sensor.intensities[8],
        //             slate.sun_sensor.intensities[9],
        //             slate.sun_sensor.intensities[10],
        //             slate.sun_sensor.intensities[11],
        //             slate.sun_sensor.intensities[12],
        //             slate.sun_sensor.intensities[13],
        //             slate.sun_sensor.intensities[14],
        //             slate.sun_sensor.intensities[15]);

        // --- Compute sun vector in body frame --- //
        slate.sun_sensor.sun_vector_valid = 
            sun_sensors_to_vector(&slate.sun_sensor, 
                    &slate.sun_sensor.sun_vector_body);

        if (slate.sun_sensor.sun_vector_valid)
        {
            LOG_DEBUG("[sun sensor] sun_vector_body = [%.3f, %.3f, %.3f]",
                      slate.sun_sensor.sun_vector_body.x, 
                      slate.sun_sensor.sun_vector_body.y,
                      slate.sun_sensor.sun_vector_body.z);

            // Update attitude filter with sun vector measurement
            // if (slate->af_is_initialized)
            // {
            //     attitude_filter_update(slate, 'S');
            // }
        } else {
            LOG_DEBUG("[sun sensor] invalid sun sensor reading");
        }


    } // end for(;;)
} // end vTaskSunSensor


