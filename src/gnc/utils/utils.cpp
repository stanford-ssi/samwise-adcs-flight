/**
 * @author Niklas Vainio, Lundeen Cahilly, Sid Manne
 * @date 2025-10-15
 *
 * This file contains general GNC utils (unit conversions, common frame
 * transforms, etc.)
 *
 */

#include "utils.h"
#include "constants.h"
#include "pico/time.h"
#include <cmath>

/**
 * @brief Return the difference between two `absolute_time_t` objects in seconds
 *
 * @param t_start
 * @param t_end
 * @return float
 */
float time_diff_seconds(absolute_time_t t_start, absolute_time_t t_end)
{
    constexpr float seconds_per_us = 1e-6f;

    // Compute dt
    uint32_t dt_us = absolute_time_diff_us(t_start, t_end);

    return (float)dt_us * seconds_per_us;
}

/**
 * @brief Returns 1.0 if x is positive, -1.0 if x is negative, and 0.0 if x is
 * zero
 */
float sign(float x)
{
    if (x == 0.0f)
        return 0.0f;

    return (x > 0.0f) ? 1.0f : -1.0f;
}

/**
 * @brief Clamp the absolute value of x to 1
 */
float clamp_abs_to_one(float x)
{
    if (x > 1.0f)
        return 1.0f;
    if (x < -1.0f)
        return -1.0f;
    return x;
}

/**
 * @brief Wrap the angle to [0, 360) degree
 */
float wrapTo360(float angle)
{
    return fmodf(fmodf(angle, 360.0f) + 360.0f, 360.0f);
}

/**
 * @brief Construct a quaternion from an axis and angle (radians)
 * @param axis      Axis of rotation (should be unit length)
 * @param angle_rad Angle of rotation in radians
 * @return quaternion
 */
quaternion quaternion_by_axis_angle(float3 axis, float angle_rad)
{
    const float half_angle = angle_rad * 0.5f;
    const float s = std::sin(half_angle);
    return quaternion{axis.x * s, axis.y * s, axis.z * s, std::cos(half_angle)};
}

/**
 * @brief Convert quaternion to modified Rodrigues parameters (MRP)
 * @param q Quaternion (x, y, z, w)
 * @return MRP (3-vector)
 */
float3 quat_to_mrp(quaternion q)
{
    float3 v = {q.x, q.y, q.z};
    float w = q.w;
    return v / (1.0f + w);
}

/**
 * @brief Wrap modified Rodrigues parameters (MRP) to shadow set if norm > 1
 * @param mrp MRP (3-vector)
 * @return Wrapped MRP (3-vector)
 */
float3 mrp_wrap_shadow_set(float3 mrp)
{
    float p = dot(mrp, mrp);
    if (p <= 1.0f)
    {
        // No need to use shadow set
        return mrp;
    }
    return -mrp / p; // wrap to shadow set if norm > 1
}

/**
 * @brief Convert modified Rodrigues parameters (MRP) to quaternion
 * (scalar-last)
 * @param mrp MRP (3-vector)
 * @return Quaternion (x, y, z, w)
 */
quaternion mrp_to_quat(float3 mrp)
{
    // Compute positive scalar component
    mrp = mrp_wrap_shadow_set(mrp); // wraps to shadow set if norm > 1
    float p = dot(mrp, mrp);
    float q_scalar = (1.0f - p) / (1.0f + p);

    quaternion q;
    q.x = mrp.x * (1.0f + q_scalar);
    q.y = mrp.y * (1.0f + q_scalar);
    q.z = mrp.z * (1.0f + q_scalar);
    q.w = p;
    return q;
}

float3x3 mrp_to_dcm(float3 mrp)
{
    float p = dot(mrp, mrp);
    float3x3 p_cross = {
        {0.0f, -mrp.z, mrp.y}, {mrp.z, 0.0f, -mrp.x}, {-mrp.y, mrp.x, 0.0f}};

    // Compute p_cross @ p_cross (matrix multiplication)
    float3x3 p_cross_squared = linalg::mul(p_cross, p_cross);

    // Compute coefficients;
    float one_plus_p_sq = (1.0f + p) * (1.0f + p);
    float coeff1 = -4.0f * (1.0f - p) / one_plus_p_sq;
    float coeff2 = 8.0f / one_plus_p_sq;

    // Compute: I - coeff1 * p_cross + coeff2 * p_cross_squared
    float3x3 identity = {
        {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    return identity + coeff1 * p_cross + coeff2 * p_cross_squared;
}

#ifdef TEST
void test_mrps()
{
    printf("Testing MRP conversions...\n");
    float3 axis = normalize(float3(1.0f, 1.0f, 0.0f));
    float angle = 90.0f * DEG_TO_RAD;
    quaternion q = quaternion_by_axis_angle(axis, angle);
    quaternion q_expected = {0.5f, 0.5f, 0.0f, std::sqrt(2.0f) / 2.0f};
    // Check quaternion is as expected
    for (int i = 0; i < 4; i++)
    {
        ASSERT_ALMOST_EQ(q[i], q_expected[i], 1e-6f);
    }
    LOG_INFO("Quaternion: [x,y,z,w]: %.6f, %.6f, %.6f, %.6f", q.x, q.y, q.z,
             q.w);
    LOG_INFO("Expected:   [x,y,z,w]: %.6f, %.6f, %.6f, %.6f", q_expected.x,
             q_expected.y, q_expected.z, q_expected.w);

    // float3 mrp = quat_to_mrp(q);
    // float3 mrp_wrapped = mrp_wrap_shadow_set(mrp);
    // quaternion q_converted = mrp_to_quat(mrp);
    // float3x3 dcm = mrp_to_dcm(mrp);
    // float3x3 dcm_from_quat = linalg::rotation_matrix(q);
}
#endif