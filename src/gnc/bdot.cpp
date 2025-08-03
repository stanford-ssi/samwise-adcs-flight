/**
 * @author The ADCS team
 * @date 2025-02-08
 */

#include "gnc/bdot.h"
#include "gnc/utils.h"
#include "macros.h"
#include "pico/stdlib.h"

// Control constrant
static constexpr float bdot_k = 1e5f;

/**
 * Return requested magnetorquer moments using proportional control (XYZ
 * order)
 *
 * @param dB
 * @param dt
 */
float3 bdot_compute_control_proportional(float3 dB, float dt)
{
    const float3 bdot = dB / dt;
    const float3 moments_raw = -bdot_k * bdot;

    // Clamp moments from 0 to 1
    return float3(clamp_abs_to_one(moments_raw.x),
                  clamp_abs_to_one(moments_raw.y),
                  clamp_abs_to_one(moments_raw.z));
}

/**
 * Return requested magnetorquer moments using bang-bang control (XYZ order)
 *
 * @param dB
 * @param dt
 */
float3 bdot_compute_control_bang_bang(float3 dB, float dt)
{
    const float3 bdot = dB / dt;

    // Return sign of moments
    return float3(sign(-bdot.x), sign(-bdot.y), sign(-bdot.z));
}

void test_bdot_control(slate_t *slate)
{
    LOG_INFO("Testing bdot...");

    // Initialize
    slate->b_field_local = {1e-6f, 0.0f, 0.0f};
    slate->b_field_local_prev = {1e-6f, 0.0f, 0.0f};
    sleep_ms(100);

    for (int i = 0; i < 10; i++)
    {
        slate->b_field_local += float3{1e-7f, 1e-7f, 1e-7f};

        const float3 dB = slate->b_field_local - slate->b_field_local_prev;
        const float dt = 0.1f;
        const float3 moments = bdot_compute_control_proportional(dB, dt);

        // Check result
        ASSERT(moments.x < 0 && moments.y < 0 && moments.z < 0);
        ASSERT(abs(moments.x) <= 1.0f && abs(moments.y) <= 1.0f &&
               abs(moments.z) <= 1.0f);

        sleep_ms(100);

        slate->b_field_local_prev = slate->b_field_local;
    }

    LOG_INFO("Bdot testing successful! :)");
}