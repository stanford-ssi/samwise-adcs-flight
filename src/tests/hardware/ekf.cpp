/**
 * @author  Chen Li
 * @date    2025-09-08
 *
 * Extended Kalman Filter test
 */

#include "macros.h"
#include "pico/stdlib.h"

#include "gnc/estimation/attitude_filter.h"
#include "gnc/models/b_field.h"
#include "gnc/models/sun_vector.h"
#include "gnc/utils/mjd.h"
#include "tasks/sensing/sensors_task.h"

#include "slate.h"

using namespace linalg::aliases;

void ekf_test(slate_t *slate)
{
    slate->gps_time = 160500.0f;
    slate->gps_lat = 37.4379532f;
    slate->gps_lon = -122.1667524f;

    slate->gps_alive = true;
    slate->gps_data_valid = true;

    LOG_DEBUG("[sensors] GPS data: Lat: %.6f, Lon: %.6f, Time: %.3f",
                    slate->gps_lat, slate->gps_lon, slate->gps_time);
    // Remember to change the hard coded date in gnc/mjd.cpp

    // MJD calculation
    compute_MJD(slate);
    LOG_DEBUG("[gnc] MJD is: %.6f", slate->MJD);

    // sun model
    compute_sun_vector_eci(slate);
    LOG_DEBUG("[gnc] sun vector [x,y,z] in eci is: %.6f, %.6f, %.6f",
             slate->sun_vector_eci[0], slate->sun_vector_eci[1],
             slate->sun_vector_eci[2]);

    // B field model
    compute_B(slate);
    LOG_DEBUG("[gnc] magnetic vector [x,y,z] in eci is: %.6f, %.6f, %.6f",
             slate->B_est_eci[0], slate->B_est_eci[1], slate->B_est_eci[2]);

    // EKF test
    int count = 0;

    slate->w_body = {0.00000f, 0.0f, 0.0f};
    for (int i = 0; i < 10; i++)
    {
        attitude_filter_propagate(slate, 0.1);

        LOG_DEBUG("%f, %f, %f, %f, %f", slate->q_eci_to_body[0],
                 slate->q_eci_to_body[1], slate->q_eci_to_body[2],
                 slate->q_eci_to_body[3], slate->attitude_covar_log_frobenius);
    }

    attitude_filter_update(slate);

    LOG_INFO("%f, %f, %f, %f, %f", ++count, slate->q_eci_to_body[0],
             slate->q_eci_to_body[1], slate->q_eci_to_body[2],
             slate->q_eci_to_body[3], slate->attitude_covar_log_frobenius);
}

void attitude_filter_software_test(slate_t *slate)
{
    static int count = 0;

    // Set test conditions for attitude filter
    slate->w_body = {0.00000f, 0.0f, 0.0f};
    slate->sun_vector_eci = {1.0f, 0.0f, 0.0f};
    slate->B_est_eci = {0.0f, 1.0f, 0.0f};

    slate->sun_vector_body = {0.0f, 1.0f, 0.0f};
    slate->b_field_local = {0.0f, 0.0f, -1.0f};

    // Propagate and print out Q + covar frobenius
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

    // ASSERT(slate->af_init_count == 1);
}
