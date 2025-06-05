/**
 * @author Niklas Vainio
 * @date 2025-06-02
 *
 * This file contains general GNC utils (unit conversions, common frame
 * transforms, etc.)
 *
 */
#pragma once

#include "linalg.h"
#include "pico/types.h"

using namespace linalg::aliases;

float time_diff_seconds(absolute_time_t t_start, absolute_time_t t_end);

float sign(float x);
float clamp_abs_to_one(float x);