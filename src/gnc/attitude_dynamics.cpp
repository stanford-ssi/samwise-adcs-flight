/**
 * @author Niklas Vainio
 * @date 2025-02-10
 */

#include "attitude_dynamics.h"
#include "constants.h"
#include "macros.h"
#include "matrix_utils.h"

// TODO - pick a good value
// TODO - move to a file for the attitude filter
// clang-format off
float af_process_noise[] = {
    0.01, 0, 0, 0, 0, 0, 0,
    0, 0.01, 0, 0, 0, 0, 0,
    0, 0, 0.01, 0, 0, 0, 0,
    0, 0, 0, 0.01, 0, 0, 0,
    0, 0, 0, 0, 0.02, 0, 0,
    0, 0, 0, 0, 0, 0.02, 0,
    0, 0, 0, 0, 0, 0, 0.02,
};
// clang-format on

static void populate_af_jacobian(float *J, quaternion q, quaternion dq,
                                 float3 w_vec, float dt, float3 I)
{
    // Return attitude filter jacobian for given parameters

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

    // clang-format-off
    float d_dq_dw[] = {dt * wx_2 * C / (2 * w_2) + (wy_2 + wz_2) * S / w_3,
                       0.5f * x * wy * (dt * C / w_2 - 2 * S / w_3),
                       0.5f * wx * wz * (dt * C / w_2 - 2 * S / w_3),
                       0.5f * wx * wy * (dt * C / w_2 - 2 * S / w_3),
                       dt * wy_2 * C / (2 * w_2) + (wx_2 + wz_2) * S / w_3,
                       0.5f * wy * wz * (dt * C / w_2 - 2 * S / w_3),
                       0.5f * wx * wz * (dt * C / w_2 - 2 * S / w_3),
                       0.5f * wy * wz * (dt * C / w_2 - 2 * S / w_3),
                       dt * wz_2 * C / (2 * w_2) + (wz_2 + wy_2) * S / w_3,
                       -0.5f * dt * wx * S / w,
                       -0.5f * dt * wy * S / w,
                       -0.5f * dt * wz * S / w};

    float d_qnew_d_dq[] = {q.w,  -q.z, q.y, q.x, q.z,  q.w,  -q.x, q.y,
                           -q.y, q.x,  q.w, q.z, -q.z, -q.y, -q.z, q.w};
    // clang-format-on

    // Compute top-right part of the jacobian
    float J_TR[12];
    mat_mul(d_qnew_d_dq, d_dq_dw, , J_TR, 4, 4, 3);

    // clang-format off
    const float J_[] = {
         dq.w,       dq.z,       -dq.y,      dq.x,       J_TR[0],      J_TR[1],       J_TR[2],
        -dq.z,       dq.w,        dq.x,      dq.y,       J_TR[3],      J_TR[4],       J_TR[5],
         dq.y,      -dq.x,        dq.w,      dq.z,       J_TR[6],      J_TR[7],       J_TR[8],
        -dq.x,      -dq.y,       -dq.z,      dq.w,       J_TR[9],     J_TR[10],      J_TR[11],
            0,          0,            0,         0,                         1,      (I[1] - I[2]) * w[2],      (I[1] - I[2]) * w[1],
            0,          0,            0,         0,      (I[2] - I[0]) * w[2],                         1,      (I[2] - I[0]) * w[0],
            0,          0,            0,         0,      (I[0] - I[1]) * w[1],      (I[0] - I[1]) * w[0],                         1,
    };
    // clang-format on

    // Copy into J
    for (int i = 0; i < 7 * 7; i++)
    {
        J[i] = J_[i];
    }
}

void propagate_attitude(slate_t *slate)
{
    // Propagate attitude in principal axes frame
    // TODO: Calculate from get_absolute_time()
    constexpr float dt = 0.1;

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
    populate_af_jacobian(J, slate->q_eci_to_principal, slate->w_principal, dt,
                         I);

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

void test_propagate_attitude(slate_t *slate)
{
    // Start at identity rotation
    slate->q_eci_to_principal = {1, 0, 0, 0};
    slate->w_principal = {1, 0.2, 0};

    for (int i = 0; i < 1000; i++)
    {
        propagate_attitude(slate);
        LOG_INFO("%f, %f, %f, %f", slate->q_eci_to_principal[0],
                 slate->q_eci_to_principal[1], slate->q_eci_to_principal[2],
                 slate->q_eci_to_principal[3]);
    }
}