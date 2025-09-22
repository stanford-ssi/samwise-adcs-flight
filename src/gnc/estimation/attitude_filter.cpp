/**
 * @author Niklas Vainio
 * @brief This file implements the main attitude EFK
 * @date 2025-05-03
 */

#include "attitude_filter.h"
#include "constants.h"
#include "gnc/utils/matrix_utils.h"
#include "linalg.h"
#include "macros.h"

// Constant matrices for process and measurement noise
// TODO - pick good.sensible values for these
// clang-format off
const float af_process_noise[7*7] = {
    0.01, 0, 0, 0, 0, 0, 0,
    0, 0.01, 0, 0, 0, 0, 0,
    0, 0, 0.01, 0, 0, 0, 0,
    0, 0, 0, 0.01, 0, 0, 0,
    0, 0, 0, 0, 0.02, 0, 0,
    0, 0, 0, 0, 0, 0.02, 0,
    0, 0, 0, 0, 0, 0, 0.02,
};

const float af_measurement_noise[4*4] = {
     0.04,  -0.01, -0.01, -0.01,
    -0.01,   0.04, -0.01, -0.01,
    -0.01,  -0.01,  0.04, -0.01,
     0.04,   0.04, -0.01, -0.01
};
// clang-format on

// Initial value of covariance matrix
const float af_initial_covar[7 * 7] = {
    1, -0.5, -0.5, -0.5, 0,    0, 0,   -0.5, 1,    -0.5, -0.5, 0, 0,
    0, -0.5, -0.5, 1,    -0.5, 0, 0,   0,    -0.5, -0.5, -0.5, 1, 0,
    0, 0,    0,    0,    0,    0, 0.2, 0,    0,    0,    0,    0, 0,
    0, 0.2,  0,    0,    0,    0, 0,   0,    0,    0.2,
};
// clang-format on

static void populate_af_jacobian(float *J, quaternion q, quaternion dq,
                                 float3 w_vec, float dt, float3 I)
{
    // Compute d(dq)/dw (useful for top right block)
    float w = length(w_vec);
    float w_2 = w * w;
    float w_3 = w_2 * w;

    float wx = w_vec.x;
    float wx_2 = wx * wx;
    float wy = w_vec.y;
    float wy_2 = wy * wy;
    float wz = w_vec.z;
    float wz_2 = wz * wz;

    float dtheta_2 = w * dt / 2;
    float C = cos(dtheta_2);
    float S = sin(dtheta_2);

    // clang-format off
    float d_dq_dw[4*3] = {
        dt * wx_2 * C / (2 * w_2) + (wy_2 + wz_2) * S / w_3,             0.5f * wx * wy * (dt * C/w_2 - 2 * S/w_3),             0.5f * wx * wz * (dt * C/w_2 - 2 * S/w_3),
        0.5f * wx * wy * (dt * C/w_2 - 2 * S/w_3),             dt * wy_2 * C / (2 * w_2) + (wx_2 + wz_2) * S / w_3,             0.5f * wy * wz * (dt * C/w_2 - 2 * S/w_3),
        0.5f * wx * wz * (dt * C/w_2 - 2 * S/w_3),                       0.5f * wy * wz * (dt * C/w_2 - 2 * S/w_3),   dt * wz_2 * C / (2 * w_2) + (wz_2 + wy_2) * S / w_3,
                          -0.5f * dt * wx * S / w,                               -0.5f * dt * wy * S / w,                               -0.5f * dt * wz * S / w
    };

    float d_qnew_d_dq[4*4] = {
        q.w,  -q.z,  q.y,  q.x,
        q.z,   q.w, -q.x,  q.y,
        -q.y,  q.x,  q.w,  q.z,
        -q.z, -q.y, -q.z,  q.w 
    };
    // clang-format on

    // Compute top-right part of the jacobian
    float J_TR[12];
    mat_mul(d_qnew_d_dq, d_dq_dw, J_TR, 4, 4, 3);

    // clang-format off
    const float J_[7*7] = {
        dq.w,       dq.z,       -dq.y,      dq.x,       J_TR[0],      J_TR[1],       J_TR[2],
        -dq.z,       dq.w,        dq.x,      dq.y,       J_TR[3],      J_TR[4],       J_TR[5],
        dq.y,      -dq.x,        dq.w,      dq.z,       J_TR[6],      J_TR[7],       J_TR[8],
        -dq.x,      -dq.y,       -dq.z,      dq.w,       J_TR[9],     J_TR[10],      J_TR[11],
        0,          0,            0,         0,                             1,      (I[1] - I[2]) * w_vec[2],      (I[1] - I[2]) * w_vec[1],
        0,          0,            0,         0,      (I[2] - I[0]) * w_vec[2],                             1,      (I[2] - I[0]) * w_vec[0],
        0,          0,            0,         0,      (I[0] - I[1]) * w_vec[1],      (I[0] - I[1]) * w_vec[0],                             1,
    };
    // clang-format on

    // Copy into J
    for (int i = 0; i < 7 * 7; i++)
    {
        J[i] = J_[i];
    }
}

/**
 * @brief Initialize the state of the attitude filter
 *
 * @param slate
 */
void attitude_filter_init(slate_t *slate)
{
    // Set q to identity, w to zero and covariance to initial value
    slate->q_eci_to_principal = {1.0f, 0.0f, 0.0f, 0.0f};
    slate->w_principal = {0.2f, -0.1f, 0.0f};

    // Copy initial covariance matrix
    for (int i = 0; i < 7 * 7; i++)
    {
        slate->attitude_covar[i] = af_initial_covar[i];
    }
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
    // Propagate attitude in principal axes frame
    const float3 w = slate->w_principal;
    const float3 w_hat = normalize(w);
    const float d_theta = dt * length(w);

    // Quaternion update
    const float3 dq_vec_part = w_hat * sin(d_theta / 2);
    quaternion dq = {dq_vec_part[0], dq_vec_part[1], dq_vec_part[2],
                     cos(d_theta / 2)};

    // Euler equations for omega
    const float3 I = SATELLITE_INERTIA;
    const float3 tau = slate->tau_principal;
    float3 w_dot = {(I[1] - I[2]) * w[1] * w[2] + tau[0] / I[0],
                    (I[2] - I[0]) * w[0] * w[2] + tau[1] / I[1],
                    (I[0] - I[1]) * w[0] * w[1] + tau[2] / I[2]};

    // Calculate attitude jacobian
    float J[7 * 7];
    populate_af_jacobian(J, slate->q_eci_to_principal, dq, slate->w_principal,
                         dt, I);

    // Perform simple dynamics update
    slate->q_eci_to_principal = qmul(slate->q_eci_to_principal, dq);
    slate->w_principal += w_dot * dt;

    // Perform jacobian update
    // C = J @ C @ J.T + Q
    float J_T[7 * 7];
    mat_transpose(J, J_T, 7, 7);

    float temp[7 * 7];
    mat_mul_square(slate->attitude_covar, J_T, temp, 7);
    mat_mul_square(J, temp, slate->attitude_covar, 7);

    mat_add(slate->attitude_covar, af_process_noise, slate->attitude_covar, 7,
            7);
}

/**
 * @brief Perform a measurement update om the attitude filter
 *
 * @param slate
 * @param q_meas_eci_to_principal
 */
void attitude_filter_update(slate_t *slate, quaternion q_meas_eci_to_principal)
{
    // TODO
}

void test_attitude_filter(slate_t *slate)
{
    // Propagate and print out Q + covar frobenius
    attitude_filter_propagate(slate, 0.01);

    LOG_INFO("%f, %f, %f, %f, %f", slate->q_eci_to_principal[0],
             slate->q_eci_to_principal[1], slate->q_eci_to_principal[2],
             slate->q_eci_to_principal[3],
             mat_frobenius(slate->attitude_covar, 7));
}