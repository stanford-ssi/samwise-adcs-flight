/**
 * @author  The ADCS Team :)
 * @date    2024-02-08
 *
 * This file defines the slate struct, a static struct which stores all data on
 * the satellite. At init time, a single instance of this struct gets statically
 * allocated, and it is referenced by all tasks and functions.
 *
 * Look up "blackboard pattern" for more info.
 */

#pragma once

#include "linalg.h"
#include "pico/types.h"

using namespace linalg::aliases;
using namespace linalg;

typedef struct samwise_slate
{
    // General world state
    float3 b_field_local;      // [T]
    float3 b_field_local_prev; // [T]
    float3 b_unit_local;       // (unit vector)  Computed from model
    float3 b_unit_eci;         // (unit vector)  Computed from model

    float3 sun_vector_eci;   // (unit vector)  Computed from model
    float3 sun_vector_local; // (unit vector)  Measured from pyramids

    float MJD;
    float latitude_rad;
    float longitude_rad;
    float altitude; // [km]
    float3 r_ecef;  // [km]
    float3 r_eci;   // [km]

    // Bdot
    float3 bdot_mu_requested; // [A * m^2]
    absolute_time_t bdot_last_ran_time;

    // Attitude propagator
    quaternion q_eci_to_body;
    float3 w_body;               // [rad s^-1] in body frame: written by IMU
    float attitude_covar[4 * 4]; // attitude covariance matrix
    float attitude_covar_log_frobenius;
    bool af_is_initialized;
    uint32_t af_init_count = 0;

    absolute_time_t af_last_ran_time;

    // Attituide control
    float3 control_torque;
    float3 reaction_wheel_speeds;

} slate_t;
