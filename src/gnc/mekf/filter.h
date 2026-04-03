/* Author:  Carson Lauer
 * Date:    28 March 2026
 *
 * MEKF Filter
 */

#include "linalg.h"
using namespace linalg;
using namespace linalg::aliases;

struct Mat3x6 {
    float data[3][6];
};

struct Mat6x6 { 
    float data[6][6];
};


class SensorFusion {
    float _var; // variance
    Mat3x6 sensitivity_mat_; // 3x6 matrix
    Mat6x6 gain_mat_; // 6x6 matrix
    float3 ref_body;
    void compute_sensitivity(const quaternion &q, const float3 &ref_eci);
    // void compute_gain(const AttitudeFilter &a);
};

class AttitudeFilter {
    Mat6x6 q_dynamics_;
    Mat6x6 dynamics_mat_;
    Mat6x6 covariance_mat_;
    SensorFusion mag_sensor_;
    SensorFusion sun_sensor_;
    public:
        // Body frame to inertial frame attitude
        quaternion quat_;
        // Error corrected angular velocity
        float3 omega_;
        // Error estimate composed of Δq and Δb
        float error_estimate_[6];
        // IMU bias estimate
        float3 bias_estimate_;
    void get_dynamics_matrix(const float3 &w_raw, float dt);    
    void propagate_covariance();
};



