/* Author:  Carson Lauer
 * Date:    28 March 2026
 *
 * MEKF Filter
 */

#pragma once

#include "gnc/utils/matrix_types.h"

#include "linalg.h"
using namespace linalg;
using namespace linalg::aliases;

class SensorFusion {
    float var_; // variance
    Mat3x6 sensitivity_mat_; // H - 3x6 matrix
    Mat6x3 gain_mat_; // K - 6x6 matrix
    Mat3x3 q_sensor_;
    float3 ref_eci_;
    public:
    float3 ref_body_;
    SensorFusion(float var);
    void set_reference(const float3 &ref_eci);
    void compute_sensitivity(const quaternion &q);
    void compute_gain(const Mat6x6 &covariance_mat);
    void propagate_covariance_sensor(Mat6x6 &covariance_mat);
    void apply_residual(const float3 &obs_body, Vec6 &error_estimate);
};

class AttitudeFilter {
    float3x3 inertia_tensor_;   // Body frame inertia
    float3x3 inertia_inverse_;  // Body frame inverse

    float var_q_; // dynamics process noise
    float var_b_; // bias process noise
    Mat6x6 q_dynamics_;     // Q_k
    Mat6x6 dynamics_mat_;   // F 
    float3 dynamics_model(const float3 &omega);
    public:
    Mat6x6 covariance_mat_; // P
    // Body frame to inertial frame attitude
    quaternion quat_;
    // Error corrected angular velocity
    float3 omega_;
    // Error estimate composed of Δq and Δb
    Vec6 error_estimate_;
    // IMU bias estimate
    float3 bias_estimate_;
    AttitudeFilter(float3x3 inertia_tensor, 
            float var_q, 
            float var_b, 
            float dt);
    void compute_dynamics_matrix(const float3 &w_raw, float dt);    
    void progagate_attitude(float dt);
    void propagate_covariance();
    void reset_error();
};



