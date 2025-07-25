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

/**
 * Apply a simple low pass filter to a changing value. Supports a wide array of
 * types with templating (floats, float3, etc).
 *
 * @tparam T        Any type that supports addition and multiplication
 * @param prev      Previous filter output
 * @param current   New input value
 * @param alpha     Percentage of new value to use. Cutoff frequency (in Hz) is
 * roughly (alpha / 2 pi * T_S) where T_S is the sampling time
 * @return T
 */
template <typename T>
T low_pass_filter(T prev, T current, float alpha)
{
    return (alpha * current) + ((1 - alpha) * prev);
}