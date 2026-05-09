/*
 * Magnetometer Kalman Filter
 * Authors: Carson Lauer & Maya Davis
 */

#include "linalg.h"
using namespace linalg::aliases;

#include "gnc/estimation/magnetometer_filter.h"
#include "gnc/utils/matrix_types.h"

MagnetometerFilter::MagnetometerFilter(float noise)
{
    noise_ = noise;
    // TODO: Initialize H, P, K, other fields
    return;
}

/*
 * The measurement model calculates the expected observation
 * from the current estimated magnetemeter parameters
 */
void MagnetometerFilter::measurement_model()
{
    // TODO: Implement h(x)
    return;
}

/*
 * update the sensitivity matrix H
 */
void MagnetometerFilter::update_sensitivity(float3 &b_body, float3 &b_reference)
{
    // TODO: Implement H update
    return;
}


/*
 * Given the sensitivity and the covariance update the gain
 */
void update_gain()
{
    // TODO: Implement K update
    return;
}

/*
 * propagates covariance P given the calculated gain and sensitivity
 */
void propagate_covariance()
{
    // TODO: Implement P update
    return;
}

Mat3x3 get_scale_matrix() {
    // TODO: return proper values for D
    return {{
        {0,0,0},
        {0,0,0},
        {0,0,0}
    }};
}

Vec3 get_bias() {
    // TODO: return proper values for b
    return {{0, 0, 0}}; 
}

