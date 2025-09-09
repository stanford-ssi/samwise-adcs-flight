/**
 * @author  Chen Li
 * @date    2025-09-08
 *
 * Extended Kalman Filter test
 */

#include "macros.h"
#include "pico/stdlib.h"

#include "../drivers/imu.h"
#include "../drivers/magnetometer.h"
#include "../gnc/attitude_filter.h"
#include "../gnc/mjd.h"
#include "../gnc/sun_pyramid_reading.h"
#include "../gnc/sun_vector.h"
#include "../gnc/world/b_field.h"
#include "../tasks/sensors_task.h"

#include "slate.h"

using namespace linalg::aliases;

void ekf_test(slate_t *slate)
{
    slate->gps_time = 160500.0f;
    // Remember to change the hard coded date in gnc/mjd.cpp

    // MJD calculation
    compute_MJD(slate);
    LOG_INFO("[gnc] MJD is: %.6f", slate->MJD);

    // sun model
    compute_sun_vector_eci(slate);
    LOG_INFO("[gnc] sun vector [x,y,z] in eci is: %.6f, %.6f, %.6f",
             slate->sun_vector_eci[0], slate->sun_vector_eci[1],
             slate->sun_vector_eci[2]);

    // B field model
    compute_B(slate);
    LOG_INFO("[gnc] magnetic vector [x,y,z] in eci is: %.6f, %.6f, %.6f",
             slate->B_est_eci[0], slate->B_est_eci[1], slate->B_est_eci[2]);

    // sensor reading

    sensors_task_dispatch(slate);

    // imu reading
    // float3 w_body_raw;
    // if (imu_get_rotation(&w_body_raw))
    // {
    //     LOG_INFO("[gnc] angular velocity raw [wx,wy,wz] in body frame is: "
    //              "%.6f, %.6f, %.6f",
    //              w_body_raw[0], w_body_raw[1], w_body_raw[2]);
    // }
    // else
    // {
    //     LOG_DEBUG("[gnc] IMU not working");
    // }

    // sun pyramid reading, no hardware now
    // slate->sun_vector_body = slate->sun_vector_eci;

    // Magnetic field reading, simple test
    // slate->b_field_local = slate->B_est_eci;
    // if (rm3100_get_reading(&slate->b_field_local))
    // {
    //     LOG_INFO("[gnc] magnetic reading raw [x,y,z] in body frame is: %.6f,
    //     "
    //              "%.6f, %.6f",
    //              slate->b_field_local[0], slate->b_field_local[1],
    //              slate->b_field_local[2]);
    // }
    // else
    // {
    //     LOG_DEBUG("[gnc] magnetometer not working");
    // }

    // EKF test

    int count = 0;

    slate->w_body = {0.00000f, 0.0f, 0.0f};
    for (int i = 0; i < 10; i++)
    {
        attitude_filter_propagate(slate, 0.1);

        LOG_INFO("%f, %f, %f, %f, %f", slate->q_eci_to_body[0],
                 slate->q_eci_to_body[1], slate->q_eci_to_body[2],
                 slate->q_eci_to_body[3], slate->attitude_covar_log_frobenius);
    }

    attitude_filter_update(slate);

    LOG_INFO("%f, %f, %f, %f, %f", ++count, slate->q_eci_to_body[0],
             slate->q_eci_to_body[1], slate->q_eci_to_body[2],
             slate->q_eci_to_body[3], slate->attitude_covar_log_frobenius);
}
