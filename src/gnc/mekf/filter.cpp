/* Author:  Carson Lauer
 * Date:    28 March 2026
 *
 * MEKF Filter
 */
#include "linalg.h"
using namespace linalg;
using namespace linalg::aliases;
#include "filter.h"
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
