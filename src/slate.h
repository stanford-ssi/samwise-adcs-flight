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
    float3 sun_vector_eci;     // (unit vector)
    float3 b_field_local;      // [T]
    float3 b_field_local_prev; // [T]

    float MJD;

    // Bdot
    float3 bdot_mu_requested; // [A * m^2]
    absolute_time_t bdot_last_ran_time;

    // Attitude propagator
    quaternion q_eci_to_principal;
    float3 w_principal;   // [rad s^-1] in principal axes frame
    float3 tau_principal; // [Nm] total torque in principal axes frame

    // TODO mat<float, 3, 3> attitude_covar;  // attitude covariance matrix

    // Attituide control
    float3 control_torque;
    float3 reaction_wheel_speeds;

} slate_t;
