/**
 * @author  The ADCS Team :3
 * @date    2024-02-08
 *
 * This file defines the slate struct, a static struct which stores all data on
 * the satellite. At init time, a single instance of this struct gets statically
 * allocated, and it is referenced by all tasks and functions.
 *
 * Look up "blackboard pattern" for more info.
 */

#pragma once

#include "adcs_packet.h"
#include "constants.h"
#include "linalg.h"
#include "macros.h"
#include "pico/types.h"
#include "scheduler/state_machine_types.h"

using namespace linalg::aliases;
using namespace linalg;

typedef struct samwise_motor_slate
{
    // ========================================================================
    //          GENERAL STATE
    // ========================================================================

    // Temporary
    float yippee;
} motor_slate_t;

