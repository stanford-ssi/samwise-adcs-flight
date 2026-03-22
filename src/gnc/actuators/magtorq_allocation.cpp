/**
 * @author Lundeen Cahilly
 * @date 2025-11-08
 *
 * This task is responsible for allocating the magnetorquer dipole moments
 * given the requested control moment
 */

#include "magtorq_allocation.h"
#include "constants.h"
#include "params.h"

static float magnetorquer_power(slate_t *slate, float3 duty_cycles)
{
    // P = I^2R
    float voltage =
        V_BATT_MAX; // [V] assume worst case - max voltage from batteries
    if (slate->power_monitor_alive)
    {
        voltage = slate->adcs_voltage;
    }
    float3 I = duty_cycles * voltage / R_MAGTORQ;
    return dot(I * I, R_MAGTORQ);
}

/**
 * @brief Allocate magnetorquer dipoles given requested control moment using
 * projection of requested moment onto magnetorquer axes, clamped by relative
 * strengths
 *
 * @param slate Pointer to the current satellite slate
 */
void allocate_magnetorquers(slate_t *slate)
{
    float3 mu_requested = slate->magtorq_requested;

    // Convert to duty-cycle space (normalize by per-axis max moment)
    float3 duty_cycle = mu_requested / MU_MAGTORQ;

    // Uniform scale to keep direction: if any axis exceeds [-1, 1],
    // scale the entire vector down by the worst offender
    float max_abs = fmaxf(fmaxf(fabsf(duty_cycle.x), fabsf(duty_cycle.y)),
                          fabsf(duty_cycle.z));
    if (max_abs > 1.0f)
    {
        duty_cycle = duty_cycle / max_abs;
    }

    // Scale uniformly if power budget is exceeded
    float power = magnetorquer_power(slate, duty_cycle);
    if (power > MAGTORQ_MAX_POWER)
    {
        float scale = sqrtf(MAGTORQ_MAX_POWER / power);
        duty_cycle = duty_cycle * scale;
    }

    float3 mu_allocated = duty_cycle * MU_MAGTORQ;

    LOG_DEBUG("[magtorq_allocation] Requested moment: (%f, %f, %f) Am^2",
              mu_requested.x, mu_requested.y, mu_requested.z);
    LOG_DEBUG("[magtorq_allocation] Allocated moment: (%f, %f, %f) Am^2",
              mu_allocated.x, mu_allocated.y, mu_allocated.z);
    LOG_DEBUG("[magtorq_allocation] Duty cycle: (%f, %f, %f)", duty_cycle.x,
              duty_cycle.y, duty_cycle.z);
    LOG_DEBUG("[magtorq_allocation] Power consumption: %f W",
              magnetorquer_power(slate, duty_cycle));

    slate->magtorq_duty_cycle = duty_cycle;
    slate->magtorq_moment = mu_allocated;
}

#ifdef TEST

static float direction_alignment(float3 a, float3 b)
{
    if (length(a) < 1e-6f && length(b) < 1e-6f)
        return 1.0f;
    return dot(normalize(a), normalize(b));
}

static bool check_duty_bounds(float3 duty)
{
    float tol = 1e-6f;
    return fabsf(duty.x) <= 1.0f + tol && fabsf(duty.y) <= 1.0f + tol &&
           fabsf(duty.z) <= 1.0f + tol;
}

/**
 * @brief Test magnetorquer allocation: direction preservation, duty cycle
 * bounds, and power limits across a range of requested moments
 */
void test_magtorq_allocation(slate_t *slate)
{
    slate->power_monitor_alive = false;
    int pass = 0;
    int fail = 0;

    float3 test_cases[9] = {
        // Nominal: well within limits
        {0.01f, 0.02f, 0.005f},
        // Symmetric
        {0.03f, 0.03f, 0.015f},
        // Exceeds z axis (should trigger uniform scaling)
        {0.0f, 0.0f, MU_MAGTORQ.z * 2.0f},
        // Exceeds all axes
        {MU_MAGTORQ.x * 3.0f, MU_MAGTORQ.y * 3.0f, MU_MAGTORQ.z * 3.0f},
        // Single axis only
        {MU_MAGTORQ.x, 0.0f, 0.0f},
        // Negative direction
        {-0.01f, -0.02f, -0.005f},
        // Large request that should hit power limit
        {MU_MAGTORQ.x, MU_MAGTORQ.y, MU_MAGTORQ.z},
        // Near-zero request
        {1e-6f, 1e-6f, 1e-6f},
    };
    int n = sizeof(test_cases) / sizeof(test_cases[0]);

    for (int i = 0; i < n; i++)
    {
        slate->magtorq_requested = test_cases[i];
        allocate_magnetorquers(slate);

        float3 req = slate->magtorq_requested;
        float3 out = slate->magtorq_moment;
        float3 duty = slate->magtorq_duty_cycle;

        // Test 1: Direction preservation
        float alignment = direction_alignment(req, out);
        bool dir_ok = alignment > 0.999f;

        // Test 2: Duty cycles within [-1, 1]
        bool duty_ok = check_duty_bounds(duty);

        // Test 3: Power within budget
        float power = magnetorquer_power(slate, duty);
        bool power_ok = power <= MAGTORQ_MAX_POWER * (1.0f + 1e-4f);

        // Test 4: Output matches duty * max moments
        float3 expected = duty * MU_MAGTORQ;
        float moment_err = length(out - expected);
        bool moment_ok = moment_err < 1e-6f;

        bool all_ok = dir_ok && duty_ok && power_ok && moment_ok;
        if (all_ok)
            pass++;
        else
            fail++;

        LOG_INFO("Test %d: %s", i + 1, all_ok ? "PASS" : "FAIL");
        LOG_INFO("  Requested:  (%f, %f, %f) Am^2", req.x, req.y, req.z);
        LOG_INFO("  Allocated:  (%f, %f, %f) Am^2", out.x, out.y, out.z);
        LOG_INFO("  Duty cycle: (%f, %f, %f)", duty.x, duty.y, duty.z);
        LOG_INFO("  Direction:  %.6f %s", alignment, dir_ok ? "OK" : "FAIL");
        LOG_INFO("  Duty bound: %s", duty_ok ? "OK" : "FAIL");
        LOG_INFO("  Power:      %.4f W %s", power, power_ok ? "OK" : "FAIL");
        LOG_INFO("  Consistency:%s", moment_ok ? "OK" : "FAIL");
    }

    LOG_INFO("[test_magtorq_allocation] Results: %d/%d passed", pass,
             pass + fail);
}
#endif