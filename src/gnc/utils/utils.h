/**
 * @author Niklas Vainio, Lundeen Cahilly
 * @date 2025-06-02
 *
 * This file contains general GNC utils (unit conversions, common frame
 * transforms, etc.)
 *
 */
#pragma once

#include "linalg.h"
#include "macros.h"
#include "pico/types.h"

using namespace linalg::aliases;

// ========================================================================
//          TIME UTILS
// ========================================================================
float time_diff_seconds(absolute_time_t t_start, absolute_time_t t_end);

// ========================================================================
//          MATH UTILS
// ========================================================================

// Basic math utils
float sign(float x);
float clamp_abs_to_one(float x);
float wrapTo360(float angle);

// Filtering
template <typename T>
T low_pass_filter(T prev, T current, float alpha)
{
    return (alpha * current) + ((1 - alpha) * prev);
}

// Quaternions
quaternion quaternion_by_axis_angle(float3 axis, float angle_rad);
float3x3 quaternion_to_dcm(quaternion q);

// Modified Rodrigues Parameters (MRPs)
float3 quat_to_mrp(quaternion q);
float3 mrp_shadow(float3 mrp);
float3 mrp_wrap_shadow_set(float3 mrp);
quaternion mrp_to_quat(float3 mrp);
float3x3 mrp_to_dcm(float3 mrp);
float3x3 cross_matrix(float3 v);

#ifdef TEST
void test_quaternions();
void test_mrps();
#endif