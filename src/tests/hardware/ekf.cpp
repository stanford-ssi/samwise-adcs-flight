/**
 * @author  Chen Li
 * @date    2025-09-08
 *
 * Extended Kalman Filter test
 */

#include "macros.h"
#include "pico/stdlib.h"
#include "tests/hardware/ekf.h"
#include "gnc/estimation/attitude_filter.h"
#include "gnc/models/b_field.h"
#include "gnc/models/sun_vector.h"
#include "gnc/utils/mjd.h"
#include "gnc/estimation/sun_sensor_to_vector.h"
#include "tasks/sensing/sensors_task.h"
#include "linalg.h"
#include "gnc/utils/utils.h"
#include <cmath>
#include "slate.h"

using namespace linalg::aliases;

void ekf_test(slate_t *slate)
{
    slate->gps_time = 130500.0f;
    // Remember to change the hard coded date in gnc/utils/mjd.cpp

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

    // calculate sun vector in body frame
    sun_sensors_to_vector(slate);

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

    // Calculate expected quaternion +X to North, +Z up
    quaternion q_expected;
    expected_quternion(slate, q_expected);
    
    LOG_INFO("[gnc] expected quaternion [x,y,z,w] is: %.6f, %.6f, %.6f, %.6f",
             q_expected[0], q_expected[1], q_expected[2], q_expected[3]);

    float3 S_local_expected = qrot(q_expected, slate->sun_vector_eci);
    float3 B_local_expected = qrot(q_expected, slate->B_est_eci);

    LOG_INFO("[gnc] actual sun vector [x,y,z] in body frame is: %.6f, %.6f, %.6f",
             slate->sun_vector_body[0], slate->sun_vector_body[1], slate->sun_vector_body[2]);
    LOG_INFO("[gnc] expected sun vector [x,y,z] in body frame is: %.6f, %.6f, %.6f",
             S_local_expected[0], S_local_expected[1], S_local_expected[2]);
    LOG_INFO("[gnc] actual B field vector [x,y,z] in body frame is: %.6f, %.6f, %.6f",
             slate->b_field_local[0], slate->b_field_local[1], slate->b_field_local[2]);         
    LOG_INFO("[gnc] expected B field vector [x,y,z] in body frame is: %.6f, %.6f, %.6f",
             B_local_expected[0], B_local_expected[1], B_local_expected[2]);
}

void expected_quternion(slate_t *slate, quaternion &q_expected)
{
    // Calculate Expected quternion when +X axis points to North and +Z axis points up

    const float GMST = wrapTo360(280.4606f + 360.9856473f * (slate->MJD - 51544.5f)) *
                       DEG_TO_RAD; // Greenwich Mean Sidereal Time
    const float ang_y = (37.42f) * DEG_TO_RAD; // rotation angle around y: lattitude
    const float ang_z = GMST + (180.0f - 122.17f) * DEG_TO_RAD; // rotation angle around z: GMST + 180 degree - longitude 
    const float Cy = cosf(ang_y);
    const float Sy = sinf(ang_y);
    const float Cz = cosf(ang_z);
    const float Sz = sinf(ang_z);

    // Attitude Matrix
    const float R[3*3] = {
        Cy*Cz, Cy*Sz, Sy,
        -Sz, Cz, 0.0f,
        -Sy*Cz, -Sy*Sz, Cy
    };

    const float r00 = R[0];
    const float r01 = R[1]; 
    const float r02 = R[2];
    const float r10 = R[3];
    const float r11 = R[4];
    const float r12 = R[5];
    const float r20 = R[6];
    const float r21 = R[7];
    const float r22 = R[8];

    const float trace = r00 + r11 + r22;
    float x, y, z, w;

    if (trace > 0.0f) {
        float s = std::sqrt(trace + 1.0f) * 2.0f; // s = 4*w
        w = 0.25f * s;
        x = (r21 - r12) / s;
        y = (r02 - r20) / s;
        z = (r10 - r01) / s;
    } else if (r00 > r11 && r00 > r22) {
        float s = std::sqrt(1.0f + r00 - r11 - r22) * 2.0f; // s = 4*x
        w = (r21 - r12) / s;
        x = 0.25f * s;
        y = (r01 + r10) / s;
        z = (r02 + r20) / s;
    } else if (r11 > r22) {
        float s = std::sqrt(1.0f + r11 - r00 - r22) * 2.0f; // s = 4*y
        w = (r02 - r20) / s;
        x = (r01 + r10) / s;
        y = 0.25f * s;
        z = (r12 + r21) / s;
    } else {
        float s = std::sqrt(1.0f + r22 - r00 - r11) * 2.0f; // s = 4*z
        w = (r10 - r01) / s;
        x = (r02 + r20) / s;
        y = (r12 + r21) / s;
        z = 0.25f * s;
    }

    // Normalize quaternion
    float n = std::sqrt(x*x + y*y + z*z + w*w);
    if (n > 0.0f) {
        x /= n; y /= n; z /= n; w /= n;
    } else {
        x = y = z = 0.0f; w = 1.0f; // fallback to identity
    }

    // Optional: enforce w >= 0 for consistent sign
    if (w < 0.0f) { x = -x; y = -y; z = -z; w = -w; }

    q_expected = {x, y, z, w};
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
