#pragma once
/*
 * Magnetometer Kalman Filter
 * Authors: Carson Lauer & Maya Davis
 */

#include "gnc/utils/matrix_types.h"

#include "linalg.h"
using namespace linalg::aliases;

class MagnetometerFilter {
    Mat9x9 P_covariance_;
    Mat1x9 K_gain_;
    Mat1x9 H_sensitivity_;

    /*
     * x = [b1, b2, b3, D11, D22, D33, D12, D13, D23]
     */
    Vec9 x_estimate_;

    float log_frob_p_;
    float noise_;

    public:
    /*
     * Object constructor
     */
    MagnetometerFilter(float noise);

    /*
     * The measurement model calculates the expected observation
     * from the current estimated magnetemeter parameters
     */
    void measurement_model();

    /*
     * update the sensitivity matrix H
     */
    void update_sensitivity(float3 &b_body, float3 &b_reference);

    /*
     * Given the sensitivity and the covariance update the gain
     */
    void update_gain();

    /*
     * propagates covariance P given the calculated gain and sensitivity
     */
    void propagate_covariance();

    Mat3x3 get_scale_matrix();
    Vec3 get_bias();
};
