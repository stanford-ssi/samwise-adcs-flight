/**
 * @author Niklas Vainio
 * @date 2025-06-02
 *
 * This file contains general GNC utils (unit conversions, common frame
 * transforms, etc.)
 *
 */

#include "utils.h"
#include "pico/time.h"

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