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

using namespace linalg;
using namespace linalg::aliases;

typedef struct samwise_slate
{
    float3 b_field_local;      // [T]
    float3 b_field_local_prev; // [T]

    absolute_time_t bdot_last_ran_time;

    float3 bdot_mu_requested; // [A * m^2]

    // Attitude
    quaternion q;
    float3 w; // [rad s^-1] in principal axes frame

} slate_t;
