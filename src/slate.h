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
    float3 b_field_local;
    float3 b_field_local_prev;

    absolute_time_t bdot_last_ran_time;

    float3 bdot_mu_requested;

    float3 control_torque;
    float3 reaction_wheel_speeds;

} slate_t;
