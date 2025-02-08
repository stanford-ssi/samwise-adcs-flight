/**
 * @author The ADCS team
 * @date 2025-02-08
 */

#include "gnc/bdot.h"
#include "macros.h"
#include "pico/stdlib.h"

constexpr float k = 1e5f;

void compute_bdot_control(slate_t *slate)
{
    // Compute dt
    uint32_t dt_us =
        absolute_time_diff_us(slate->bdot_last_ran_time, get_absolute_time());
    slate->bdot_last_ran_time = get_absolute_time();

    float dt = (float)dt_us / 1e6f;

    // Compute dB/dt
    float3 dB = slate->b_field_local - slate->b_field_local_prev;
    float3 bdot = dB / dt;

    // Compute mu
    slate->bdot_mu_requested = -k * bdot;

    // Save previous field
    slate->b_field_local_prev = slate->b_field_local;
}

void test_bdot_control(slate_t *slate)
{
    LOG_INFO("Testing bdot...");

    // Initialize
    slate->b_field_local = {1e-6f, 0.0f, 0.0f};
    slate->b_field_local_prev = {1e-6f, 0.0f, 0.0f};
    slate->bdot_last_ran_time = get_absolute_time();
    sleep_ms(100);

    for (int i = 0; i < 10; i++)
    {
        slate->b_field_local += float3{1e-7f, 1e-7f, 1e-7f};

        compute_bdot_control(slate);

        // Check result
        ASSERT(slate->bdot_mu_requested[0] < 0);
        ASSERT(slate->bdot_mu_requested[1] < 0);
        ASSERT(slate->bdot_mu_requested[2] < 0);

        sleep_ms(100);
    }

    LOG_INFO("Bdot testing successful! :)");
}