/**
 * @author Niklas Vainio
 * @date 2025-02-10
 */

#include "attitude_dynamics.h"
#include "constants.h"
#include "macros.h"

// static void populate_af_jacobian(quaternion q, float3 w, float dt, float3 I)
// {
//     // Return attitude filter jacobian for given parameters
//     return {
//         {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
//         {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0},
//         {0, 0, 0, 0, 0, 0, 0},
//     };
// }

void propagate_attitude(slate_t *slate)
{
    // Propagate attitude in principal axes frame
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
    // slate->attitude_covar = jacobian(q, w, dt, I);

    // Perform simple dynamics update
    slate->q_eci_to_principal = qmul(slate->q_eci_to_principal, dq);
    slate->w_principal += w_dot * dt;
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