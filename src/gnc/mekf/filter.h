/* Author:  Carson Lauer
 * Date:    28 March 2026
 *
 * MEKF Filter
 */

#pragma once

#include "linalg.h"
using namespace linalg;
using namespace linalg::aliases;

struct Vec6 {
    float data[6];
};

struct Vec3 {
    float data[3];
};

struct Mat3x3 {
    float data[3][3];   
};


struct Mat6x3 {
    float data[6][3];
};

struct Mat3x6 {
    float data[3][6];
};

struct Mat6x6 { 
    float data[6][6];
};


class SensorFusion {
    float _var; // variance
    Mat3x6 sensitivity_mat_; // H - 3x6 matrix
    Mat6x3 gain_mat_; // K - 6x6 matrix
    Mat3x3 q_sensor_;
    float3 ref_body_;
    void compute_sensitivity(const quaternion &q, const float3 &ref_eci);
    void compute_gain(const Mat6x6 &covariance_mat);
    void propagate_covariance_sensor(Mat6x6 &covariance_mat);
    void apply_residual(const float3 &obs_body, Vec6 &error_estimate);
};

class AttitudeFilter {
    float3x3 inertia_tensor_;   // Body frame inertia
    float3x3 inertia_inverse_;  // Body frame inverse

    Mat6x6 q_dynamics_;     // Q_k
    Mat6x6 dynamics_mat_;   // F 
    Mat6x6 covariance_mat_; // P
    SensorFusion mag_sensor_; 
    SensorFusion sun_sensor_;
    float3 dynamics_model(const float3 &omega);
    public:
        // Body frame to inertial frame attitude
        quaternion quat_;
        // Error corrected angular velocity
        float3 omega_;
        // Error estimate composed of Δq and Δb
        Vec6 error_estimate_;
        // IMU bias estimate
        float3 bias_estimate_;
    void compute_dynamics_matrix(const float3 &w_raw, float dt);    
    void progagate_attitude(float dt);
    void propagate_covariance();
    void reset_error();
};



