#include "FreeRTOS.h"
#include "task.h"

#include "macros.h"

#include "apps/adcs_new/tasks/tasks.h"
#include "apps/adcs_new/params.h"

#include "apps/adcs_new/slate.h"

#include "gnc/utils/utils.h"
#include "pico/time.h"

extern slate_t slate;

// This pretty much needs to be written anew
void vTaskGPS(void *) {
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_GPS));
    } // end for(;;)
} // end vTaskGPS

void vTaskIMU(void *) {
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_IMU));
        TASK_LOOP_MS(100);

        if (!slate.IMU_data.imu_alive)
        {
            LOG_DEBUG("[sensor] Skipping IMU due to invalid initialization!");
            return;
        }

        // Read gyro data
        bool gyro_result = imu_get_rotation(&slate.IMU_data.w_body_raw);

        if (gyro_result)
        {
            // Apply low pass filter - we run at 50 Hz so alpha = 2*pi*fc*dt
            // For fc = 1 Hz cutoff, dt = 0.02s (50 Hz): alpha = 2*pi*1*0.02 =
            // 0.1257
            constexpr float imu_lpf_alpha = 0.125663706144;
            slate.IMU_data.w_body =
                low_pass_filter(slate.IMU_data.w_body, 
                        slate.IMU_data.w_body_raw,
                        imu_lpf_alpha);

            // Update magnitude
            slate.IMU_data.w_mag = length(slate.IMU_data.w_body);

            LOG_DEBUG("[sensor] w_body = [%.5f, %.5f, %.5f]", 
                    slate.IMU_data.w_body[0],
                    slate.IMU_data.w_body[1],
                    slate.IMU_data.w_body[2]);
        }

        // Read accelerometer data separately
        bool accel_result = imu_get_accel(&slate.IMU_data.a_body);

        if (accel_result) {
            LOG_DEBUG("[sensor] a_body = [%.6f, %.6f, %.6f] km/s^2",
                    slate.IMU_data.a_body.x,
                    slate.IMU_data.a_body.y,
                    slate.IMU_data.a_body.z);
        }

        slate.IMU_data.imu_data_valid = gyro_result;

    } // end for(;;)
} // end vTaskIMU

void vTaskMagnetometer(void *) {
    // Track last read cycle start for 100ms period
    absolute_time_t last_mag_read_start = {0}; 

    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_MAGNETOMETER));
        TASK_LOOP_MS(20);

        rm3100_error_t result =
            rm3100_get_reading(&slate.MAG_data.b_body, 
                    &slate.MAG_data.b_body_raw);

        slate.MAG_data.magnetometer_data_valid = (result == RM3100_OK);
        slate.MAG_data.b_body_read_time = get_absolute_time();
        last_mag_read_start = get_absolute_time();

        if (result != RM3100_OK) {
            LOG_ERROR("[sensor] Error reading magnetometer");
        }
        // Update attitude filter with magnetometer measurement
        if (result == RM3100_OK) {
            LOG_DEBUG("[sensor] b_body = [%.3f, %.3f, %.3f]",
                    slate.MAG_data.b_body.x,
                    slate.MAG_data.b_body.y,
                    slate.MAG_data.b_body.z);
        }
    } // end for(;;)
}

void vTaskSunSensor(void *) {


}


