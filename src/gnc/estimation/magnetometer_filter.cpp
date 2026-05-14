/*
 * Magnetometer Kalman Filter
 * Authors: Carson Lauer & Maya Davis
 */

#include "linalg.h"
using namespace linalg::aliases;
using namespace linalg; 

#include "gnc/estimation/magnetometer_filter.h"
#include "gnc/utils/matrix_types.h"
#include "gnc/utils/matrix_utils.h"

/*
 * constructor
 */
MagnetometerFilter::MagnetometerFilter(float noise)
{
    noise_ = noise;
    // TODO: Initialize H, P, K, other fields
    P_covariance_ = {{{1,0,0,0,0,0,0,0,0},
                      {0,1,0,0,0,0,0,0,0},
                      {0,0,1,0,0,0,0,0,0},
                      {0,0,0,1,0,0,0,0,0},
                      {0,0,0,0,1,0,0,0,0},
                      {0,0,0,0,0,1,0,0,0},
                      {0,0,0,0,0,0,1,0,0},
                      {0,0,0,0,0,0,0,1,0},
                      {0,0,0,0,0,0,0,0,1}}};    // Mat9x9
    // H_sensitivity_ = {{{0,0,0,0,0,0,0,0,0}}};   // Mat1x9
    // K_gain_ = {{{0,0,0,0,0,0,0,0,0}}};      // Mat1x9 
    x_estimate_ = {{0,0,0,0,0,0,0,0,0}};   // Vec9
    y = 0.0f;
    // update_gain(); 
    return;
}

/*
 * The measurement model calculates the expected observation
 * from the current estimated magnetometer parameters
 */
void MagnetometerFilter::measurement_model(float3 &B_body) 
// (5.13.26) removed parameter float3 &x, replaced with float3 &B_body
{
    // TODO: Implement h(x)
    // update y 
    // update K
    // update H
    // update x 
    // update P

    // hk(x) = -SkT(E)(D vector) + 2BkT(I3 + D matrix)b - ||b||**2
    Vec3 b = get_bias(); 
    Mat3x3 Dm = get_scale_matrix(); 
    Vec6 Dv = get_scale_vector();
    Mat3x3 I3 = {{{1,0,0},{0,1,0},{0,0,1}}};
    Mat3x3 out = {{{0,0,0},{0,0,0},{0,0,0}}};
    
    return; 
}

/*
 * todo
 * 1. implement h(x)
 */


/*
 * Updates y using the magnitude of measured B and R matrices
 */
void MagnetometerFilter::update_y(float3 &R_reference, float3 &B_body)
{
    // yk = ||Bk||**2 - ||Rk||**2
    y = length2(B_body) + length2(R_reference); 
    return;
}

void MagnetometerFilter::update_x()  // removed parameters
{
    // TODO: implement update x
    // x(k+1) = xk + Kk(y(k+1) - h(k+1)(xk)) 
    // float multiplier = y - measurement_model(x_estimate_);
    return; 
}

/*
 * update the sensitivity matrix H
 */
void MagnetometerFilter::update_sensitivity(float3 &B_body)
{
    // TODO: Implement H update

    // H(x) = [2B^T(I3 + D) - 2b^T    -S^TMed(Dhat) + 2J]
    // 6.4.1
    // 6.4.4
    

    
    return;
}


/*
 * Given the sensitivity and the covariance update the gain
 */
void MagnetometerFilter::update_gain()
{
    // TODO: Implement K update
    // Kk = PkHT(k+1)(xk)[H(k+1)xkPkHT(k+1)xk + σ^2(k+1)xk]-1

    return;
}

/*
 * propagates covariance P given the calculated gain and sensitivity
 */
void MagnetometerFilter::propagate_covariance()
{
    // TODO: Implement P update
    return;
}

/*
 * returns proper values for D
 */
Mat3x3 MagnetometerFilter::get_scale_matrix() {
    // x = [b1, b2, b3, D11, D22, D33, D12, D13, D23]
    return {{
        {x_estimate_.data[3],   x_estimate_.data[6],x_estimate_.data[7]},
        {0,                     x_estimate_.data[4],x_estimate_.data[8]},
        {0,                     0,                  x_estimate_.data[5]}
    }};
}

Vec6 MagnetometerFilter::get_scale_vector() {
    // x = [b1, b2, b3, D11, D22, D33, D12, D13, D23]
    return {{x_estimate_.data[3], x_estimate_.data[4], x_estimate_.data[5], 
        x_estimate_.data[6], x_estimate_.data[7], x_estimate_.data[8]}};
}

/*
 * returns proper values for b
 */
Vec3 MagnetometerFilter::get_bias() {
    return {{x_estimate_.data[0], x_estimate_.data[1], x_estimate_.data[2]}};
    // return {{0, 0, 0}}; 
}

