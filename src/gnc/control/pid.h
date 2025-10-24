/**
 * @author Lundeen Cahilly
 * @date 2025-10-24
 *
 * This file contains the PID control law for attitude control using quaternions
 * See page 10 of this paper https://s3vi.ndc.nasa.gov/ssri-kb/static/resources/Attitude%20Determination%20&%20Control%20System%20Design%20and%20Implementation.pdf
 */
#pragma once


#include "linalg.h"
#include "slate.h"

using namespace linalg::aliases;

void compute_control_torque_pid(slate_t *slate);