/**
 * @author Niklas Vainio
 * @brief This file implements the main attitude EFK
 * @date 2025-05-03
 *
 * Information largely sourced from
 * https://ahrs.readthedocs.io/en/latest/filters/ekf.html#initial-values
 */

#include "attitude_filter.h"
#include "constants.h"
#include "gnc/utils/matrix_utils.h"
#include "linalg.h"
#include "macros.h"

// Sep 4 testing note: Change of vari names: b_unit_local -> b_field_local
// Sep 4 testing note: Change of vari names: sun_vector_local -> sun_vector_body
// Sep 5 testing note: Change of vari names: b_unit_eci ->B_est_eci

// State is just the quaternion
#define AF_STATE_SIZE (4)

// Measurement is local sun and magnetic field vectors
#define AF_MEASUREMENT_SIZE (6)

// Identity matrix instantiation
// clang-format off
const float I_4[AF_STATE_SIZE * AF_STATE_SIZE] = {
    1, 0, 0, 0, 
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
};
// clang-format on

// Measurement covariance matrix
// clang-format off
const float af_measurement_covariance[AF_MEASUREMENT_SIZE * AF_MEASUREMENT_SIZE] = {
    SUN_SENSOR_STD, 0, 0, 0, 0, 0,
    0, SUN_SENSOR_STD, 0, 0, 0, 0,
    0, 0, SUN_SENSOR_STD, 0, 0, 0,
    0, 0, 0, MAG_SENSOR_STD, 0, 0,
    0, 0, 0, 0, MAG_SENSOR_STD, 0,
    0, 0, 0, 0, 0, MAG_SENSOR_STD,
};
// clang-format on

static void populate_af_jacobian(float *J, float3 w_vec, float dt)
{
    // Populate J with jacobian
    // Since the state only contains the quaternion and we just propagate based
    // on the gyro, this has a relatively simple closed form

    const float wx = w_vec.x;
    const float wy = w_vec.y;
    const float wz = w_vec.z;

    // clang-format off
    const float J_[AF_STATE_SIZE * AF_STATE_SIZE] = {
                 1,  dt/2 * wz, -dt/2 * wy,  dt/2 * wx,
        -dt/2 * wz,          1,  dt/2 * wx,  dt/2 * wy,
         dt/2 * wy, -dt/2 * wx,          1,  dt/2 * wz,
        -dt/2 * wx, -dt/2 * wy, -dt/2 * wz,          1, 
    };
    // clang-format on

    // Copy into J
    for (int i = 0; i < AF_STATE_SIZE * AF_STATE_SIZE; i++)
    {
        J[i] = J_[i];
    }
}

static void populate_af_process_noise(float *Q, quaternion q, float dt)
{
    // Populate Q with process noise

    // clang-format off
    const float W[AF_STATE_SIZE * 3] = {
         q.w, -q.z,  q.y,
         q.z,  q.w, -q.x,
        -q.y,  q.x,  q.w,
        -q.x, -q.y, -q.z,
    };
    // clang-format on

    float W_T[3 * AF_STATE_SIZE];
    mat_transpose(W, W_T, AF_STATE_SIZE, 3);

    float Q_[AF_STATE_SIZE * AF_STATE_SIZE];
    mat_mul(W, W_T, Q_, AF_STATE_SIZE, 3, AF_STATE_SIZE);

    // Copy into Q, multiplying by noise and dt^2
    const float scale = (GYRO_STD_DEV * GYRO_STD_DEV) * (dt * dt);

    for (int i = 0; i < AF_STATE_SIZE * AF_STATE_SIZE; i++)
    {
        Q[i] = scale * Q_[i];
    }
}

static void populate_measurement_jacobian(float *H, float3 S, float3 B,
                                          quaternion q)
{
    // Populate H with measurement jacobian
    // S and B are the actual vectors in ECI

    // clang-format off
    const float H_[AF_MEASUREMENT_SIZE * AF_STATE_SIZE] = {
        S.x*q.x + S.y*q.y + S.z*q.z, -S.x*q.y + S.y*q.x - S.z*q.w, -S.x*q.z + S.y*q.w + S.z*q.x,  S.x*q.w + S.y*q.z - S.z*q.y,
        S.x*q.y - S.y*q.x + S.z*q.w,  S.x*q.x + S.y*q.y + S.z*q.z, -S.x*q.w - S.y*q.z + S.z*q.y, -S.x*q.z + S.y*q.w + S.z*q.x,
        S.x*q.z - S.y*q.w - S.z*q.x,  S.x*q.w + S.y*q.z - S.z*q.y,  S.x*q.x + S.y*q.y + S.z*q.z,  S.x*q.y - S.y*q.x + S.z*q.w,
         
        B.x*q.x + B.y*q.y + B.z*q.z, -B.x*q.y + B.y*q.x - B.z*q.w, -B.x*q.z + B.y*q.w + B.z*q.x,  B.x*q.w + B.y*q.z - B.z*q.y,
        B.x*q.y - B.y*q.x + B.z*q.w,  B.x*q.x + B.y*q.y + B.z*q.z, -B.x*q.w - B.y*q.z + B.z*q.y, -B.x*q.z + B.y*q.w + B.z*q.x,
        B.x*q.z - B.y*q.w - B.z*q.x,  B.x*q.w + B.y*q.z - B.z*q.y,  B.x*q.x + B.y*q.y + B.z*q.z,  B.x*q.y - B.y*q.x + B.z*q.w,
    };
    // clang-format on

    for (int i = 0; i < AF_MEASUREMENT_SIZE * AF_STATE_SIZE; i++)
    {
        H[i] = 2 * H_[i];
    }

    return;
}

static void populate_expected_measurement(float *z, quaternion q_eci_to_body,
                                          float3 S_eci, float3 B_eci)
{
    // Populate z with the expected measurement given S and B fields in ECI
    float3 S_local_expected = qrot(q_eci_to_body, S_eci);
    float3 B_local_expected = qrot(q_eci_to_body, B_eci);

    z[0] = S_local_expected.x;
    z[1] = S_local_expected.y;
    z[2] = S_local_expected.z;
    z[3] = B_local_expected.x;
    z[4] = B_local_expected.y;
    z[5] = B_local_expected.z;
}

static void populate_actual_measurement(float *z, float3 S, float3 B)
{
    // Populate z with the actual Sun + B field measurements
    z[0] = S.x;
    z[1] = S.y;
    z[2] = S.z;
    z[3] = B.x;
    z[4] = B.y;
    z[5] = B.z;
}

// ****************************************************************************
// ***************     PUBLIC FUNCTIONS                ************************
// ****************************************************************************

/**
 * @brief Initialize the state of the attitude filter
 *
 * @param slate
 */
void attitude_filter_init(slate_t *slate)
{
    // Set q to identity, and covariance to initial value (4x4 identity)
    slate->q_eci_to_body = {0.0f, 0.0f, 0.0f, 1.0f};

    for (int i = 0; i < AF_STATE_SIZE * AF_STATE_SIZE; i++)
    {
        slate->attitude_covar[i] = I_4[i];
    }

    slate->af_is_initialized = true;
    slate->af_init_count++;
    LOG_INFO("Initialized attitude filter! (%d times so far)",
             slate->af_init_count);
}

/**
 * @brief Propagate the attitude filter following satellite dynamics over a time
 * dt
 *
 * @param slate
 * @param dt
 */
void attitude_filter_propagate(slate_t *slate, float dt)
{
    // Initialize if not already
    if (!slate->af_is_initialized)
    {
        attitude_filter_init(slate);
    }

    // Propagate attitude using gyro
    const float3 w = slate->w_body;
    quaternion dq;

    if (length(w) > 1e-10)
    {
        const float3 w_hat = normalize(w);
        const float d_theta = dt * length(w);

        // Quaternion update
        const float3 dq_vec_part = w_hat * sin(d_theta / 2);
        dq = {dq_vec_part[0], dq_vec_part[1], dq_vec_part[2], cos(d_theta / 2)};
    }
    else
    {
        dq = {0.0f, 0.0f, 0.0f, 1.0f};
    }

    // Calculate attitude jacobian and process noise matrices
    float J[AF_STATE_SIZE * AF_STATE_SIZE];
    float Q[AF_STATE_SIZE * AF_STATE_SIZE];
    populate_af_jacobian(J, slate->w_body, dt);
    populate_af_process_noise(Q, slate->q_eci_to_body, dt);

    // Update state and covariance matrix
    // C = J @ C @ J.T + Q
    slate->q_eci_to_body = qmul(slate->q_eci_to_body, dq);

    float J_T[AF_STATE_SIZE * AF_STATE_SIZE];
    mat_transpose(J, J_T, AF_STATE_SIZE, AF_STATE_SIZE);

    float temp[AF_STATE_SIZE * AF_STATE_SIZE];
    mat_mul_square(slate->attitude_covar, J_T, temp, AF_STATE_SIZE);
    mat_mul_square(J, temp, slate->attitude_covar, AF_STATE_SIZE);

    mat_add(slate->attitude_covar, Q, slate->attitude_covar, AF_STATE_SIZE,
            AF_STATE_SIZE);

    // Compute log frobenius
    slate->attitude_covar_log_frobenius =
        mat_log_frobenius(slate->attitude_covar, AF_STATE_SIZE);

    // Check for NaN and de-initialize if so
    if (mat_contains_nan(slate->attitude_covar, AF_STATE_SIZE, AF_STATE_SIZE))
    {
        LOG_ERROR(
            "Attitude covariance contains NaN after propagate: resetting!");
        attitude_filter_init(slate);
    }
}

/**
 * @brief Perform a measurement update on the attitude filter
 *
 * @param slate
 */
void attitude_filter_update(slate_t *slate)
{
    if (!slate->af_is_initialized)
    {
        LOG_ERROR("Attempt to call update on uninitialized attitude filter! "
                  "Exiting...");
        return;
    }

    // Compute expected and actual measurements
    float z_expected[AF_MEASUREMENT_SIZE];
    float z_actual[AF_MEASUREMENT_SIZE];

    populate_expected_measurement(z_expected, slate->q_eci_to_body,
                                  slate->sun_vector_eci, slate->B_est_eci);
    populate_actual_measurement(z_actual, slate->sun_vector_body,
                                slate->b_field_local);

    float z_diff[AF_MEASUREMENT_SIZE];
    mat_sub(z_actual, z_expected, z_diff, AF_MEASUREMENT_SIZE, 1);

    // Compute innovation (S) and Kalman gain (K)
    // S = H @ C @ H.T + R
    // K = C @ H.T @ inv(S)
    float H[AF_MEASUREMENT_SIZE * AF_STATE_SIZE];
    populate_measurement_jacobian(H, slate->sun_vector_eci, slate->B_est_eci,
                                  slate->q_eci_to_body);

    float H_T[AF_STATE_SIZE * AF_MEASUREMENT_SIZE];
    mat_transpose(H, H_T, AF_MEASUREMENT_SIZE, AF_STATE_SIZE);

    float tmp[AF_MEASUREMENT_SIZE * AF_STATE_SIZE];
    float S[AF_MEASUREMENT_SIZE * AF_MEASUREMENT_SIZE];
    mat_mul(H, slate->attitude_covar, tmp, AF_MEASUREMENT_SIZE, AF_STATE_SIZE,
            AF_STATE_SIZE);
    mat_mul(tmp, H_T, S, AF_MEASUREMENT_SIZE, AF_STATE_SIZE,
            AF_MEASUREMENT_SIZE);
    mat_add(S, af_measurement_covariance, S, AF_MEASUREMENT_SIZE,
            AF_MEASUREMENT_SIZE);

    float S_inv[AF_MEASUREMENT_SIZE * AF_MEASUREMENT_SIZE];
    mat_inverse(S, S_inv, AF_MEASUREMENT_SIZE);

    float K[AF_STATE_SIZE * AF_MEASUREMENT_SIZE];
    float tmp2[AF_STATE_SIZE * AF_MEASUREMENT_SIZE];
    mat_mul(H_T, S_inv, tmp2, AF_STATE_SIZE, AF_MEASUREMENT_SIZE,
            AF_MEASUREMENT_SIZE);
    mat_mul(slate->attitude_covar, tmp2, K, AF_STATE_SIZE, AF_STATE_SIZE,
            AF_MEASUREMENT_SIZE);

    // Update quaternion (with normalization)
    float dq[AF_STATE_SIZE];
    mat_mul(K, z_diff, dq, AF_STATE_SIZE, AF_MEASUREMENT_SIZE, 1);

    quaternion dq_quat;
    dq_quat.x = dq[0];
    dq_quat.y = dq[1];
    dq_quat.z = dq[2];
    dq_quat.w = dq[3];

    slate->q_eci_to_body = normalize(slate->q_eci_to_body + dq_quat);

    // Update covariance matrix
    // C' = (I - K @ H) @ C
    float KH[AF_STATE_SIZE * AF_STATE_SIZE];
    mat_mul(K, H, KH, AF_STATE_SIZE, AF_MEASUREMENT_SIZE, AF_STATE_SIZE);

    float tmp3[AF_STATE_SIZE * AF_STATE_SIZE];
    mat_sub(I_4, KH, tmp3, AF_STATE_SIZE, AF_STATE_SIZE);

    float tmp4[AF_STATE_SIZE * AF_STATE_SIZE];
    mat_mul_square(tmp3, slate->attitude_covar, tmp4, AF_STATE_SIZE);

    for (int i = 0; i < AF_STATE_SIZE * AF_STATE_SIZE; i++)
    {
        slate->attitude_covar[i] = tmp4[i];
    }

    // Compute log frobenius
    slate->attitude_covar_log_frobenius =
        mat_log_frobenius(slate->attitude_covar, AF_STATE_SIZE);

    // Check for NaN and de-initialize if so
    if (mat_contains_nan(dq, AF_STATE_SIZE, 1))
    {
        LOG_ERROR("Attitude quat contains NaN after update: resetting!");
        attitude_filter_init(slate);
    }

    if (mat_contains_nan(slate->attitude_covar, AF_STATE_SIZE, AF_STATE_SIZE))
    {
        LOG_ERROR("Attitude covariance contains NaN after update: resetting!");
        attitude_filter_init(slate);
    }
}

int count = 0;
;
void test_attitude_filter(slate_t *slate)
{
    // Propagate and print out Q + covar frobenius
    slate->w_body = {0.00000f, 0.0f, 0.0f};
    slate->sun_vector_eci = {1.0f, 0.0f, 0.0f};
    slate->B_est_eci = {0.0f, 1.0f, 0.0f};

    slate->sun_vector_body = {0.0f, 1.0f, 0.0f};
    slate->b_field_local = {0.0f, 0.0f, -1.0f};

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