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

constexpr Mat6x6 I6 = {{
    {1, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 1}
}};

/* 
 * Takes as input the expected reference vector v, and returns
 * the sensitivity matrix H given that expected reference vector.
 */
void SensorFusion::compute_sensitivity(const quaternion &q_body,
        const float3 &ref_eci) 
{
    ref_body_ = qrot(qconj(q_body), ref_eci);
    float3 v = ref_body_;
    sensitivity_mat_ =  {{
        {0, -v[2], v[1], 0, 0, 0},
        {v[2], 0, -v[0], 0, 0, 0},
        {-v[1], v[0], 0, 0, 0, 0}
    }};
}

/*
 * In Python:
 * K = P @ H.T @ np.linalg.inv( H @ P @ H.T + factor*var_r0 * np.identity(3))
 * factor is an arbitrary constant for noise tuning
 *
 * Note: Stack is 81 words
 */
void SensorFusion::compute_gain(const Mat6x6 &covariance_mat) {
    Mat3x6 h_mul_p; 
    Mat6x3 h_transpose;
    Mat3x3 h_mul_p_mul_ht;
    Mat3x3 pre_inverse;
    Mat3x3 inverse_output;
    Mat6x3 p_mul_ht;

    // H.T (3x6) -> (6x3)
    mat_transpose(sensitivity_mat_.data[0],
            h_transpose.data[0],
            3, 6);
    
    // H(3x6) @ P (6x6) -> (3x6)
    mat_mul(sensitivity_mat_.data[0], 
            covariance_mat.data[0],
            h_mul_p.data[0],
            3, 6, 6);

    // H @ P (3x6) @ H.T (6x3) -> 3x3
    mat_mul(h_mul_p.data[0],
            h_transpose.data[0],
            h_mul_p_mul_ht.data[0],
            3, 6, 3);

    // H @ P @ H.T (3x3) + q_sensor_ (3x3)
    mat_add(h_mul_p_mul_ht.data[0],
            q_sensor_.data[0],
            pre_inverse.data[0],
            3, 3);

    // inv( H @ P @ H.T + q_sensor_) -> 3x3
    mat_inverse(pre_inverse.data[0],
            inverse_output.data[0],
            3);

    // P (6x6) @ H.T (6x3) -> (6x3)
    mat_mul(sensitivity_mat_.data[0],
            h_transpose.data[0],
            p_mul_ht.data[0],
            6, 6, 3); 

    // K = P @ H.T (6x3) @ inv(...) (3x3) -> (6x3)
    mat_mul(p_mul_ht.data[0],
            inverse_output.data[0],
            gain_mat_.data[0],
            6, 3, 3);
}

// P = (I6 - K @ H ) @ P 
void SensorFusion::propagate_covariance_sensor(Mat6x6 &covariance_mat) {
    Mat6x6 k_mul_h;
    Mat6x6 sum;
    Mat6x6 output;

    // K (6x3) @ H (3x6) -> 6x6
    mat_mul(gain_mat_.data[0],
            sensitivity_mat_.data[0],
            k_mul_h.data[0],
            6, 3, 6);

    // I6 - K @ H 
    mat_sub(k_mul_h.data[0],
            I6.data[0],
            sum.data[0],
            6, 6);
    
    mat_mul(sum.data[0],
            covariance_mat.data[0],
            output.data[0],
            6, 6, 6);

    covariance_mat = output;
}


/*
 * e0 = b0k - r0k
 * dx = dx + K0 @ (e0 - H0 @ dx)
 */
void SensorFusion::apply_residual(const float3 &obs_body, Vec6 &error_estimate) {
    Vec3 h_mul_dx;
    Vec3 measurement_residual; // Also called innovation
    Vec6 error_estimate_temp = error_estimate;
    Vec6 error_update;

    // e
    float3 error = obs_body - ref_body_;

    // H (3x6) @ dx (6x1);
    mat_mul(sensitivity_mat_.data[0],
            error_estimate.data,
            h_mul_dx.data,
            3, 6, 1);

    // e - H @ dx
    mat_sub(&error.x,
            h_mul_dx.data,
            measurement_residual.data,
            6, 1);

    // K (6x3) @ ( e - H @ dx) (3x1)
    mat_mul(gain_mat_.data[0],
            measurement_residual.data,
            error_update.data,
            6, 3, 1);

    // dx + K @ (e - H @ dx)
    mat_add(error_estimate_temp.data,
            error_update.data,
            error_estimate.data,
            6, 1);
}


/*
 * Takes as input the bias-corrected angular velocity and returns
 * the discrete dynamics matrix F.
 */
void AttitudeFilter::compute_dynamics_matrix(const float3 &w_raw, float dt) {
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

void AttitudeFilter::reset_error() {
    float3 dq_vec = {error_estimate_.data[0],
        error_estimate_.data[1], 
        error_estimate_.data[2]};

    float3 db = {error_estimate_.data[3],
        error_estimate_.data[4],
        error_estimate_.data[5]};

    quaternion dq = rotation_quat(normalize(dq_vec), length(dq_vec)); 

    quat_ = qmul(quat_, dq);

    bias_estimate_ += db;  

    error_estimate_ = {{0, 0, 0, 0, 0, 0}};

}
