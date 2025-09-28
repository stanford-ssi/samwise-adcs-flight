/**
 * @author  Chen Li
 * @date    2025-09-08
 *
 * Extended Kalman Filter test
 */

#include "tests/hardware/ekf.h"
#include "gnc/estimation/attitude_filter.h"
#include "gnc/estimation/sun_sensor_to_vector.h"
#include "gnc/models/b_field.h"
#include "gnc/models/sun_vector.h"
#include "gnc/utils/mjd.h"
#include "gnc/utils/utils.h"
#include "linalg.h"
#include "macros.h"
#include "pico/stdlib.h"
#include "slate.h"
#include "tasks/sensing/sensors_task.h"
#include <cmath>

using namespace linalg::aliases;

static const float STANFORD_LAT = 37.4379532f;
static const float STANFORD_LON = -122.1667524f;
static const float TEST_GPS_TIME = 160500.0f;
static const float EKF_TIMESTEP = 0.1f;
static const int EKF_PROPAGATION_STEPS = 10;

void ekf_test(slate_t *slate)
{
    // Write to slate in case GPS not connected / fixed
    slate->gps_time = TEST_GPS_TIME;
    slate->gps_lat = STANFORD_LAT;
    slate->gps_lon = STANFORD_LON;
    slate->gps_alive = true;
    slate->gps_data_valid = true;

    LOG_DEBUG("[ekf_test] GPS data: Lat: %.6f, Lon: %.6f, Time: %.3f",
              slate->gps_lat, slate->gps_lon, slate->gps_time);

    compute_MJD(slate);
    compute_sun_vector_eci(slate);
    compute_B(slate);
    sun_sensors_to_vector(slate);

    LOG_DEBUG("[ekf_test] MJD: %.6f", slate->MJD);

    slate->w_body = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < EKF_PROPAGATION_STEPS; i++)
    {
        attitude_filter_propagate(slate, EKF_TIMESTEP);
    }

    attitude_filter_update(slate);

    LOG_DEBUG("[ekf_test] %.6f, %.6f, %.6f, %.6f, %.6f",
              slate->q_eci_to_body[0], slate->q_eci_to_body[1],
              slate->q_eci_to_body[2], slate->q_eci_to_body[3],
              slate->attitude_covar_log_frobenius);

    quaternion q_expected;
    expected_quaternion(slate, q_expected);

    LOG_DEBUG(
        "[ekf_test] Expected quaternion [x,y,z,w]: %.6f, %.6f, %.6f, %.6f",
        q_expected[0], q_expected[1], q_expected[2], q_expected[3]);
}

void expected_quaternion(slate_t *slate, quaternion &q_expected)
{
    const float gmst =
        wrapTo360(280.4606f + 360.9856473f * (slate->MJD - 51544.5f)) *
        DEG_TO_RAD;
    const float lat_rad = STANFORD_LAT * DEG_TO_RAD;
    const float lon_offset_rad = gmst + (180.0f - STANFORD_LON) * DEG_TO_RAD;
    const float cy = cosf(lat_rad);
    const float sy = sinf(lat_rad);
    const float cz = cosf(lon_offset_rad);
    const float sz = sinf(lon_offset_rad);

    const float R[9] = {cy * cz, cy * sz,  sy,       -sz, cz,
                        0.0f,    -sy * cz, -sy * sz, cy};

    const float trace = R[0] + R[4] + R[8];
    float x, y, z, w;

    if (trace > 0.0f)
    {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        w = 0.25f * s;
        x = (R[7] - R[5]) / s;
        y = (R[2] - R[6]) / s;
        z = (R[3] - R[1]) / s;
    }
    else if (R[0] > R[4] && R[0] > R[8])
    {
        const float s = std::sqrt(1.0f + R[0] - R[4] - R[8]) * 2.0f;
        w = (R[7] - R[5]) / s;
        x = 0.25f * s;
        y = (R[1] + R[3]) / s;
        z = (R[2] + R[6]) / s;
    }
    else if (R[4] > R[8])
    {
        const float s = std::sqrt(1.0f + R[4] - R[0] - R[8]) * 2.0f;
        w = (R[2] - R[6]) / s;
        x = (R[1] + R[3]) / s;
        y = 0.25f * s;
        z = (R[5] + R[7]) / s;
    }
    else
    {
        const float s = std::sqrt(1.0f + R[8] - R[0] - R[4]) * 2.0f;
        w = (R[3] - R[1]) / s;
        x = (R[2] + R[6]) / s;
        y = (R[5] + R[7]) / s;
        z = 0.25f * s;
    }

    const float norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (norm > 0.0f)
    {
        x /= norm;
        y /= norm;
        z /= norm;
        w /= norm;
    }
    else
    {
        x = y = z = 0.0f;
        w = 1.0f;
    }

    if (w < 0.0f)
    {
        x = -x;
        y = -y;
        z = -z;
        w = -w;
    }

    q_expected = {x, y, z, w};
}

void attitude_filter_software_test(slate_t *slate)
{
    slate->w_body = {0.0f, 0.0f, 0.0f};
    slate->sun_vector_eci = {1.0f, 0.0f, 0.0f};
    slate->B_est_eci = {0.0f, 1.0f, 0.0f};
    slate->sun_vector_body = {0.0f, 1.0f, 0.0f};
    slate->b_field_local = {0.0f, 0.0f, -1.0f};

    for (int i = 0; i < EKF_PROPAGATION_STEPS; i++)
    {
        attitude_filter_propagate(slate, EKF_TIMESTEP);
    }

    attitude_filter_update(slate);

    LOG_DEBUG("[ekf_test] %.6f, %.6f, %.6f, %.6f, %.6f",
              slate->q_eci_to_body[0], slate->q_eci_to_body[1],
              slate->q_eci_to_body[2], slate->q_eci_to_body[3],
              slate->attitude_covar_log_frobenius);
}