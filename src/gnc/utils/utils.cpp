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
 * @brief Convert quaternion to direction cosine matrix (DCM)
 * @param q Quaternion (x, y, z, w)
 * @return DCM (3x3 matrix)
 */
float3x3 quaternion_to_dcm(quaternion q)
{
    float3x3 dcm;

    float qx2 = q.x * q.x;
    float qy2 = q.y * q.y;
    float qz2 = q.z * q.z;
    float qw2 = q.w * q.w;

    // Match linalg library convention (qxdir, qydir, qzdir) with active
    // rotation
    dcm[0][0] = qw2 + qx2 - qy2 - qz2;
    dcm[0][1] = 2.0f * (q.x * q.y + q.z * q.w);
    dcm[0][2] = 2.0f * (q.z * q.x - q.y * q.w);

    dcm[1][0] = 2.0f * (q.x * q.y - q.z * q.w);
    dcm[1][1] = qw2 - qx2 + qy2 - qz2;
    dcm[1][2] = 2.0f * (q.y * q.z + q.x * q.w);

    dcm[2][0] = 2.0f * (q.z * q.x + q.y * q.w);
    dcm[2][1] = 2.0f * (q.y * q.z - q.x * q.w);
    dcm[2][2] = qw2 - qx2 - qy2 + qz2;

    return dcm;
}

/**
 * @brief Convert quaternion to modified Rodrigues parameters (MRP)
 * @param q Quaternion (x, y, z, w)
 * @return MRP (3-vector)
 */
float3 quat_to_mrp(quaternion q)
{
    if (q.w < 0.0f) // Ensure scalar part is non-negative
    {
        q = -q;
    }
    float3 v = {q.x, q.y, q.z};
    float w = q.w;

    return v / (1.0f + w);
}

/**
 * @brief Compute the shadow set of modified Rodrigues parameters (MRP)
 * @param mrp MRP (3-vector)
 * @return Shadow set MRP (3-vector)
 */
float3 mrp_shadow(float3 mrp)
{
    float p = dot(mrp, mrp);
    return -mrp / p; // wrap to shadow set
}

/**
 * @brief Wrap modified Rodrigues parameters (MRP) to shadow set if norm > 1/3
 * @param mrp MRP (3-vector)
 * @return Wrapped MRP (3-vector)
 */
float3 mrp_wrap_shadow_set(float3 mrp)
{
    float p = dot(mrp, mrp);
    if (p > 1.0f)
    {
        return -mrp / p; // wrap to shadow set if norm > 1
    }
    return mrp; // No need to use shadow set
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

    quaternion q = {mrp.x * (1.0f + q_scalar), mrp.y * (1.0f + q_scalar),
                    mrp.z * (1.0f + q_scalar), q_scalar};
    return q;
}

/**
 * @brief Compute the cross product matrix (also known as the skew-symmetric
 * matrix) for a given 3-vector.
 * @param v Input 3-vector
 * @return 3x3 cross product matrix
 */
float3x3 cross_matrix(float3 v)
{
    return float3x3{{0.0f, -v.z, v.y}, {v.z, 0.0f, -v.x}, {-v.y, v.x, 0.0f}};
}

/**
 * @brief Convert modified Rodrigues parameters (MRP) to direction cosine
 * matrix (DCM)
 * @param mrp MRP (3-vector)
 * @return DCM (3x3 matrix)
 */
float3x3 mrp_to_dcm(float3 mrp)
{
    float p = dot(mrp, mrp);
    float3x3 p_cross = cross_matrix(mrp);

    // Compute p_cross @ p_cross (matrix multiplication)
    float3x3 p_cross_squared = mul(p_cross, p_cross);

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
void test_quaternions()
{
    printf("\n><=><=><=><=><= Testing quaternion utilities... "
           "><=><=><=><=><=\n");
    // ========================================================================
    //      TEST BASIC QUATERNION CONSTRUCTION
    // ========================================================================
    LOG_INFO("Testing basic quaternion construction!");
    // Construct a quaterion representing a 90 degree rotation about the
    // vector (1, 1, 0)
    float3 axis = normalize(float3(1.0f, 1.0f, 0.0f));
    float angle = 90.0f * DEG_TO_RAD;
    quaternion q = quaternion_by_axis_angle(axis, angle);
    quaternion q_expected = {0.5f, 0.5f, 0.0f, std::sqrt(2.0f) / 2.0f};
    for (int i = 0; i < 4; i++)
    {
        ASSERT_ALMOST_EQ(q[i], q_expected[i], 1e-6f);
    }
}

void test_mrps()
{
    printf("\n><=><=><=><=><= Testing MRP conversion utilities... "
           "><=><=><=><=><=\n");
    // ========================================================================
    //      TEST BASIC QUATERNION TO MRP CONVERSIONS
    // ========================================================================
    LOG_INFO("Testing basic quaternion to mrp conversions!");
    // Construct a quaterion representing a 90 degree rotation about the
    // vector (1, 1, 0) - scalar part will be positive
    float3 axis = normalize(float3(1.0f, 1.0f, 0.0f));
    float angle = 90.0f * DEG_TO_RAD;
    quaternion q = quaternion_by_axis_angle(axis, angle);
    quaternion q_expected = {0.5f, 0.5f, 0.0f, std::sqrt(2.0f) / 2.0f};
    for (int i = 0; i < 4; i++)
    {
        ASSERT_ALMOST_EQ(q[i], q_expected[i], 1e-6f);
    }

    // Convert to MRP
    float3 p = quat_to_mrp(q);
    float3 p_expected = {0.292894f, 0.292894f,
                         0.0f}; // Calculated w/ mathematica
    for (int i = 0; i < 3; i++)
    {
        ASSERT_ALMOST_EQ(p[i], p_expected[i], 1e-6f);
    }
    // Convert back to quaternion and compare to original
    quaternion q_converted = mrp_to_quat(p);
    for (int i = 0; i < 4; i++)
    {
        ASSERT_ALMOST_EQ(q[i], q_converted[i], 1e-6f);
    }

    // ========================================================================
    //      TEST SHADOW SET WRAPPING (>180º rotations)
    // ========================================================================
    LOG_INFO("Testing shadow set wrapping for quaternion to MRP!");
    float3 axis_shadow = normalize(float3(0.0f, 1.0f, 0.0f));
    float angle_shadow = 181.0f * DEG_TO_RAD;
    quaternion q_shadow = quaternion_by_axis_angle(axis_shadow, angle_shadow);
    quaternion q_shadow_expected = {0.0f, std::sin(angle_shadow / 2.0f), 0.0f,
                                    std::cos(angle_shadow / 2.0f)};
    for (int i = 0; i < 4; i++)
    {
        ASSERT_ALMOST_EQ(q_shadow[i], q_shadow_expected[i], 1e-6f);
    }

    // Convert to MRP
    float3 p_shadow = quat_to_mrp(q_shadow);
    float3 p_shadow_expected = {0.0f, -0.991311,
                                0.0f}; // Calculated w/ mathematica
    for (int i = 0; i < 3; i++)
    {
        ASSERT_ALMOST_EQ(p_shadow[i], p_shadow_expected[i], 1e-6f);
    }

    // Confirm we apply shadow set wrapping when going mrp -> quaternion
    quaternion q_shadow_converted =
        mrp_to_quat(p_shadow); // convert back, we should now have q => -q (same
                               // rotation, positive scalar part)
    quaternion q_shadow_converted_expected = {
        0.0f, -0.999962f, 0.0f, 0.00872655f}; // Calculated w/ mathematica
    for (int i = 0; i < 4; i++)
    {
        ASSERT_ALMOST_EQ(q_shadow_converted[i], q_shadow_converted_expected[i],
                         1e-6f);
        ASSERT_ALMOST_EQ(q_shadow_expected[i], q_shadow_converted[i] * -1,
                         1e-6f); // Should also equal -q_shadow_expected (same
                                 // rotation -q = q)
    }

    // ========================================================================
    //      TEST HIGH ROTATION SHADOW SETS (>350º rotations)
    // ========================================================================
    LOG_INFO("Testing shadow set wrapping for very high rotations!");
    float3 axis_high = normalize(float3(1.0f, 1.0f, 1.0f));
    float angle_high = 359.0f * DEG_TO_RAD;
    quaternion q_high = quaternion_by_axis_angle(axis_high, angle_high);
    float3 p_high = quat_to_mrp(q_high);
    quaternion q_high_converted = mrp_to_quat(p_high);
    for (int i = 0; i < 4; i++)
    {
        ASSERT_ALMOST_EQ(q_high[i], -1 * q_high_converted[i], 1e-6f);
    }

    // ========================================================================
    //      TESTING MRP TO DCM CONVERSION
    // ========================================================================
    LOG_INFO("Testing MRP to DCM conversion!");
    // Test 90 degree rotation about (1, 1, 0) axis
    float3 mrp_test = float3(0.292894f, 0.292894f, 0.0f);
    float3x3 dcm = mrp_to_dcm(mrp_test);
    float3x3 dcm_expected = {
        {0.5f, 0.5f, -0.7071068f},
        {0.5f, 0.5f, 0.7071068f},
        {0.7071068f, -0.7071068f, 0.0f}}; // Calculated w/ mathematica
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            ASSERT_ALMOST_EQ(dcm[i][j], dcm_expected[i][j], 1e-3f);
        }
    }

    // Test rotating a vector using the DCM
    float3 v = float3(1.0f, 0.0f, 0.0f);
    float3 v_rotated = mul(dcm, v);
    float3 v_rotated_expected = mul(dcm_expected, v);
    for (int i = 0; i < 3; i++)
    {
        ASSERT_ALMOST_EQ(v_rotated[i], v_rotated_expected[i], 1e-3f);
    }
}
#endif