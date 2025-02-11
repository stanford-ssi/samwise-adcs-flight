/**
 * @author Niklas Vainio
 * @date 2025-02-10
 */

#include "attitude_dynamics.h"

// TODO: Move to constants file
constexpr float3 SATELLITE_INERTIA = {0.01461922201, 0.0412768466,
                                      0.03235309961}; // [kg m^2]

struct q_and_omega
{
    quaternion q;
    float3 w;
};

static q_and_omega attitude_dynamics(quaternion q, float3 w, float3 I,
                                     float3 tau)
{
    // Compute dynamics matrix for the current rotation
    // NOTE: linalg.h matrix literals are column major, so transposed
    // compared to sims
    float4x4 stm = {{0, w[0], w[1], w[2]},
                    {-w[0], 0, -w[2], w[1]},
                    {-w[1], w[2], 0, -w[0]},
                    {-w[2], -w[1], w[0], 0}};

    float4 q_dot = 0.5f * mul(stm, q);

    float3 w_dot = {(I[1] - I[2]) * w[1] * w[2] + tau[0] / I[0],
                    (I[2] - I[0]) * w[0] * w[2] + tau[1] / I[1],
                    (I[0] - I[1]) * w[0] * w[1] + tau[2] / I[2]};

    return q_and_omega{.q = q_dot, .w = w_dot};
}

void propagate_attitude(slate_t *slate)
{
    // Perform RK4 integration
    constexpr float3 tau = {0, 0, 0};
    constexpr dt = 0.1;

    q_and_omega x_dot;

    q_and_omega k1 =
        attitude_dynamics(slate->q, slate->w_principal, SATELLITE_INERTIA, tau);
    quaternion q1 = normalize(slate->q + dt / 2.0f * k1.q);
    float3 w1 = slate->w + dt / 2.0f * k1->w;

    q_and_omega k2 = attitude_dynamics(q1, w1, SATELLITE_INERTIA, tau);
    quaternion q2 = normalize(slate->q + dt / 2.0f * k2->q);
    float3 w2 = slate->w + dt / 2.0f * k2->w;

    q_and_omega k3 = attitude_dynamics(q2, w2, SATELLITE_INERTIA, tau);
    quaternion q3 = normalize(slate->q + dt * k3->q);
    float3 w3 = slate->w + dt / 2.0f * k3->w;
}

void test_propagate_attitude(slate_t *slate)
{
    // TODO
}