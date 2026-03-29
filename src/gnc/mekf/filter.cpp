/* Author:  Carson Lauer
 * Date:    28 March 2026
 *
 * MEKF Filter
 */
#include "linalg.h"
using namespace linalg::aliases;

// Sensitivity Matrix is a 3 row, 6 col matrix
typedef mat<float, 3, 6> float3x6; 

// Covariance Matrix F is a 6 row by 6 col matrix
// Also gain matrix K
typedef mat<float, 6, 6> float6x6;

class AttitudeFilter {
    float6x6 covariance_mat;
    SensorFilter mag_sensor;
    SensorFilter sun_sensor;
    public:
        // Body frame to inertial frame attitude
        float4 quat;
        // Error corrected angular velocity
        float3 omega;
        // Error estimate composed of Δq and Δb
        float6 error_estimate;
        // IMU bias estimate
        float3 bias_estimate;
    
    void update_covariance
};

class SensorFilter {
    float var; // variance
    float3x6 sensitivity_mat;
    float6x6 gain_mat;
    public:
        float3 v_reference;
        float3 v_observed;
    void compute_sensitivity(&self, const AttitudeFilter &a);
    void compute_gain(&self, const AttitudeFilter &a);
}

float3x3 get_skew_symmetric(float3 v) {
    return {{0, -v[2], v[1]},
            {v[2], 0, -v[0]},
            {-v[1], v[0], 0}};
}

/* 
 * Takes as input the expected reference vector v, and returns
 * the sensitivity matrix H given that expected reference vector.
 */
float3x6 get_sensitivity_matrix(float3 v) {
    return {{0, -v[2], v[1], 0, 0, 0},
            {v[2], 0, -v[0], 0, 0, 0},
            {-v[1], v[0], 0, 0, 0, 0}};
}

/*
 * Takes as input the bias-corrected angular velocity and returns
 * the discrete dynamics matrix F.
 */
float6x6 get_dynamics_matrix(float3 w, float dt) {
    return {{1, -dt * v[2], dt * v[1], -dt, 0, 0},
            {dt * v[2], 1, -dt * v[0], 0, -dt, 0},
            {-dt * v[1], dt * v[0], 1, 0, 0, -dt},
            {0, 0, 0, 1, 0, 0},
            {0, 0, 0, 0, 1, 0},
            {0, 0, 0, 0, 0, 1}};
}

class MekfFilter: public AttitudeFilter {
    SensorReference magnetometer;
    SensorReference sun_sensor;
}
