/* Author:  Carson Lauer
 * Date:    28 March 2026
 *
 * MEKF Filter
 */
#include "linalg.h"
using namespace linalg;
using namespace linalg::aliases;

#include "filter.h"

#include "gnc/utils/matrix_utils.h"
/* 
 * Takes as input the expected reference vector v, and returns
 * the sensitivity matrix H given that expected reference vector.
 */
void SensorFusion::compute_sensitivity(const quaternion &q_body,
        const float3 &ref_eci) 
{
    float3 v = ref_eci;
    sensitivity_mat_ =  {{
        {0, -v[2], v[1], 0, 0, 0},
        {v[2], 0, -v[0], 0, 0, 0},
        {-v[1], v[0], 0, 0, 0, 0}
    }};
}

/*
 * Takes as input the bias-corrected angular velocity and returns
 * the discrete dynamics matrix F.
 */
void AttitudeFilter::get_dynamics_matrix(const float3 &w_raw, float dt) {
    float3 w = w_raw - bias_estimate_;
    omega_ = w;
    dynamics_mat_ =  {{
        {1, -dt * w[2], dt * w[1], -dt, 0, 0},
        {dt * w[2], 1, -dt * w[0], 0, -dt, 0},
        {-dt * w[1], dt * w[0], 1, 0, 0, -dt},
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1}
        }};
}

// Stack size: 36 + 36 + 36 = 108
void AttitudeFilter::propagate_covariance() {
    Mat6x6 dynamics_transpose;
    mat_transpose(dynamics_mat_.data[0], dynamics_transpose.data[0], 6, 6);

    Mat6x6 p_step1;

    mat_mul_square(dynamics_mat_.data[0], covariance_mat_.data[0], p_step1.data[0], 6);

    Mat6x6 p_step2;

    mat_mul_square(p_step1.data[0], dynamics_transpose.data[0], p_step2.data[0], 6);

    mat_add(p_step2.data[0], q_dynamics_.data[0], covariance_mat_.data[0], 6, 6);
}
