/* Author:  Carson Lauer
 * Date:    28 March 2026
 *
 * MEKF Filter
 */

#include "linalg.h"
using namespace linalg;
using namespace linalg::aliases;

// Sensitivity Matrix is a 3 row, 6 col matrix
typedef mat<float, 3, 6> float3x6; 

// Covariance Matrix F is a 6 row by 6 col matrix
// Also gain matrix K
typedef mat<float, 6, 6> float6x6;

class AttitudeFilter {
    float6x6 dynamics_mat;
    float6x6 covariance_mat;
    SensorFilter mag_sensor;
    SensorFilter sun_sensor;
    public:
        // Body frame to inertial frame attitude
        quaternion quat;
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
    float3 ref_body;
    void compute_sensitivity(&self, const quaternion &q, const float3 ref_eci);
    void compute_gain(&self, const AttitudeFilter &a);
}


