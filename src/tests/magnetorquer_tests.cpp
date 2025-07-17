/**
 * @author  Lundeen Cahilly
 * @date    2025-07-16
 *
 * Test functions for magnetorquer pwm driver validation
 */

#include "magnetorquer_tests.h"
#include "../drivers/magnetometer.h"
#include "../drivers/magnetorquer.h"
#include "macros.h"
#include "pico/stdlib.h"
#include <math.h>

// Test state tracking
typedef enum
{
    TEST_IDLE,
    TEST_4_1_BOOT,
    TEST_4_2_PWM_0_PERCENT,
    TEST_4_2_PWM_50_PERCENT,
    TEST_4_2_PWM_100_PERCENT,
    TEST_4_3_POLARITY_X_POS,
    TEST_4_3_POLARITY_X_NEG,
    TEST_4_3_POLARITY_Y_POS,
    TEST_4_3_POLARITY_Y_NEG,
    TEST_4_3_POLARITY_Z_POS,
    TEST_4_3_POLARITY_Z_NEG,
    TEST_COMPLETE
} test_state_t;

static test_state_t current_test_state = TEST_IDLE;
static uint32_t test_start_time = 0;
static bool tests_complete = false;

// Test configuration
#define TEST_DURATION_MS (3000)    // 3 seconds per test
#define TEST_SETTLE_TIME_MS (1000) // 1 second settle time
#define PWM_TEST_CURRENT_LIMIT (200)

/**
 * Test 4.1: Magnetorquer Boot Test
 * Enable magnetorquers, check voltage and current measurement data
 */
static void test_4_1_magnetorquer_boot(void)
{
    LOG_INFO(
        "[mag_test] 4.1 - Testing magnetorquer boot and basic functionality");

    // Initialize PWM system
    init_pwm();

    // Test with minimal duty cycle to check basic operation
    pwm_error_t result = do_pwm(10, 10, 10, PWM_TEST_CURRENT_LIMIT);

    if (result == PWM_OK)
    {
        LOG_INFO("[mag_test] 4.1 - PWM initialization SUCCESS");
        LOG_INFO(
            "[mag_test] 4.1 - Check voltage across magnetorquers manually");
        LOG_INFO(
            "[mag_test] 4.1 - Check current measurement (CM) data via mux");
    }
    else
    {
        LOG_ERROR("[mag_test] 4.1 - PWM initialization FAILED with error: %d",
                  result);
    }

    // Stop PWM after test
    stop_pwm();
}

/**
 * Test 4.2: PWM Duty Cycle Test
 * Test magnetorquers at 0%, 50%, and 100% duty cycle with magnetometer readings
 */
static void test_4_2_pwm_duty_cycles(uint8_t duty_percent)
{
    LOG_INFO("[mag_test] 4.2 - Testing PWM at %d%% duty cycle", duty_percent);

    // Calculate duty cycle value (-128 to 128 range)
    int8_t duty_value = 0;
    if (duty_percent == 50)
    {
        duty_value = 64; // 50% of 128
    }
    else if (duty_percent == 100)
    {
        duty_value = 128; // 100%
    }
    // 0% remains 0

    // Apply PWM to X-axis only for this test
    pwm_error_t pwm_result = do_pwm(duty_value, 0, 0, PWM_TEST_CURRENT_LIMIT);

    if (pwm_result != PWM_OK)
    {
        LOG_ERROR("[mag_test] 4.2 - PWM setup failed with error: %d",
                  pwm_result);
        return;
    }

    // Wait for magnetic field to stabilize
    sleep_ms(TEST_SETTLE_TIME_MS);

    // Take magnetometer reading
    float3 mag_field;
    rm3100_error_t mag_result = rm3100_get_reading(&mag_field);

    if (mag_result == RM3100_OK)
    {
        float magnitude =
            sqrt(mag_field.x * mag_field.x + mag_field.y * mag_field.y +
                 mag_field.z * mag_field.z);

        LOG_INFO("[mag_test] 4.2 - %d%% duty: Mag field = [%.2f, %.2f, %.2f] "
                 "uT, |B| = %.2f uT",
                 duty_percent, mag_field.x, mag_field.y, mag_field.z,
                 magnitude);
    }
    else
    {
        LOG_ERROR("[mag_test] 4.2 - Magnetometer read failed with error: %d",
                  mag_result);
    }

    // Stop PWM
    stop_pwm();
}

/**
 * Test 4.3: Polarity Test
 * Verify magnetic moment direction matches requested direction
 */
static void test_4_3_polarity_test(const char *axis, int8_t x_duty,
                                   int8_t y_duty, int8_t z_duty)
{
    LOG_INFO("[mag_test] 4.3 - Testing %s polarity [%d, %d, %d]", axis, x_duty,
             y_duty, z_duty);

    // Take baseline magnetometer reading with PWM off
    float3 baseline_field;
    rm3100_error_t baseline_result = rm3100_get_reading(&baseline_field);
    if (baseline_result != RM3100_OK)
    {
        LOG_ERROR("[mag_test] 4.3 - Baseline magnetometer read failed");
        return;
    }

    // Apply PWM
    pwm_error_t pwm_result =
        do_pwm(x_duty, y_duty, z_duty, PWM_TEST_CURRENT_LIMIT);
    if (pwm_result != PWM_OK)
    {
        LOG_ERROR("[mag_test] 4.3 - PWM setup failed with error: %d",
                  pwm_result);
        return;
    }

    // Wait for field to stabilize
    sleep_ms(TEST_SETTLE_TIME_MS);

    // Take magnetometer reading with PWM on
    float3 active_field;
    rm3100_error_t active_result = rm3100_get_reading(&active_field);
    if (active_result != RM3100_OK)
    {
        LOG_ERROR("[mag_test] 4.3 - Active magnetometer read failed");
        stop_pwm();
        return;
    }

    // Calculate field difference (induced field)
    float3 induced_field = {active_field.x - baseline_field.x,
                            active_field.y - baseline_field.y,
                            active_field.z - baseline_field.z};

    float induced_magnitude = sqrt(induced_field.x * induced_field.x +
                                   induced_field.y * induced_field.y +
                                   induced_field.z * induced_field.z);

    LOG_INFO("[mag_test] 4.3 - %s: Baseline = [%.2f, %.2f, %.2f] uT", axis,
             baseline_field.x, baseline_field.y, baseline_field.z);
    LOG_INFO("[mag_test] 4.3 - %s: Active   = [%.2f, %.2f, %.2f] uT", axis,
             active_field.x, active_field.y, active_field.z);
    LOG_INFO(
        "[mag_test] 4.3 - %s: Induced  = [%.2f, %.2f, %.2f] uT, |B| = %.2f uT",
        axis, induced_field.x, induced_field.y, induced_field.z,
        induced_magnitude);

    // Basic polarity check - the dominant component should match the expected
    // axis
    bool polarity_correct = false;
    if (x_duty != 0)
    {
        polarity_correct = (x_duty > 0 && induced_field.x > 0) ||
                           (x_duty < 0 && induced_field.x < 0);
        LOG_INFO("[mag_test] 4.3 - X-axis polarity: %s",
                 polarity_correct ? "CORRECT" : "INCORRECT");
    }
    if (y_duty != 0)
    {
        polarity_correct = (y_duty > 0 && induced_field.y > 0) ||
                           (y_duty < 0 && induced_field.y < 0);
        LOG_INFO("[mag_test] 4.3 - Y-axis polarity: %s",
                 polarity_correct ? "CORRECT" : "INCORRECT");
    }
    if (z_duty != 0)
    {
        polarity_correct = (z_duty > 0 && induced_field.z > 0) ||
                           (z_duty < 0 && induced_field.z < 0);
        LOG_INFO("[mag_test] 4.3 - Z-axis polarity: %s",
                 polarity_correct ? "CORRECT" : "INCORRECT");
    }

    // Stop PWM
    stop_pwm();
}

void magnetorquer_tests_init(void)
{
    LOG_INFO("[mag_test] Initializing magnetorquer test module");

    // Initialize magnetometer for field measurements
    rm3100_error_t mag_result = rm3100_init();
    if (mag_result != RM3100_OK)
    {
        LOG_ERROR("[mag_test] Failed to initialize magnetometer for testing");
    }
    else
    {
        LOG_INFO("[mag_test] Magnetometer initialized for testing");
    }

    current_test_state = TEST_IDLE;
    tests_complete = false;
}

bool magnetorquer_tests_dispatch(void)
{
    if (tests_complete)
    {
        return true; // Tests already completed
    }

    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    switch (current_test_state)
    {
        case TEST_IDLE:
            LOG_INFO("[mag_test] Starting magnetorquer test sequence");
            current_test_state = TEST_4_1_BOOT;
            test_start_time = current_time;
            break;

        case TEST_4_1_BOOT:
            test_4_1_magnetorquer_boot();
            current_test_state = TEST_4_2_PWM_0_PERCENT;
            test_start_time = current_time;
            break;

        case TEST_4_2_PWM_0_PERCENT:
            if (current_time - test_start_time >= TEST_DURATION_MS)
            {
                test_4_2_pwm_duty_cycles(0);
                current_test_state = TEST_4_2_PWM_50_PERCENT;
                test_start_time = current_time;
            }
            break;

        case TEST_4_2_PWM_50_PERCENT:
            if (current_time - test_start_time >= TEST_DURATION_MS)
            {
                test_4_2_pwm_duty_cycles(50);
                current_test_state = TEST_4_2_PWM_100_PERCENT;
                test_start_time = current_time;
            }
            break;

        case TEST_4_2_PWM_100_PERCENT:
            if (current_time - test_start_time >= TEST_DURATION_MS)
            {
                test_4_2_pwm_duty_cycles(100);
                current_test_state = TEST_4_3_POLARITY_X_POS;
                test_start_time = current_time;
            }
            break;

        case TEST_4_3_POLARITY_X_POS:
            if (current_time - test_start_time >= TEST_DURATION_MS)
            {
                test_4_3_polarity_test("X+", 100, 0, 0);
                current_test_state = TEST_4_3_POLARITY_X_NEG;
                test_start_time = current_time;
            }
            break;

        case TEST_4_3_POLARITY_X_NEG:
            if (current_time - test_start_time >= TEST_DURATION_MS)
            {
                test_4_3_polarity_test("X-", -100, 0, 0);
                current_test_state = TEST_4_3_POLARITY_Y_POS;
                test_start_time = current_time;
            }
            break;

        case TEST_4_3_POLARITY_Y_POS:
            if (current_time - test_start_time >= TEST_DURATION_MS)
            {
                test_4_3_polarity_test("Y+", 0, 100, 0);
                current_test_state = TEST_4_3_POLARITY_Y_NEG;
                test_start_time = current_time;
            }
            break;

        case TEST_4_3_POLARITY_Y_NEG:
            if (current_time - test_start_time >= TEST_DURATION_MS)
            {
                test_4_3_polarity_test("Y-", 0, -100, 0);
                current_test_state = TEST_4_3_POLARITY_Z_POS;
                test_start_time = current_time;
            }
            break;

        case TEST_4_3_POLARITY_Z_POS:
            if (current_time - test_start_time >= TEST_DURATION_MS)
            {
                test_4_3_polarity_test("Z+", 0, 0, 100);
                current_test_state = TEST_4_3_POLARITY_Z_NEG;
                test_start_time = current_time;
            }
            break;

        case TEST_4_3_POLARITY_Z_NEG:
            if (current_time - test_start_time >= TEST_DURATION_MS)
            {
                test_4_3_polarity_test("Z-", 0, 0, -100);
                current_test_state = TEST_COMPLETE;
                test_start_time = current_time;
            }
            break;

        case TEST_COMPLETE:
            if (current_time - test_start_time >= TEST_DURATION_MS)
            {
                LOG_INFO("[mag_test] ===== MAGNETORQUER TEST SEQUENCE COMPLETE "
                         "=====");
                LOG_INFO("[mag_test] Check logs above for results. Manual "
                         "verification required for:");
                LOG_INFO(
                    "[mag_test] - Voltage measurements across magnetorquers");
                LOG_INFO("[mag_test] - Current measurement (CM) data via mux");
                LOG_INFO("[mag_test] - Polarity verification using "
                         "magnetometer readings");
                tests_complete = true;
                return true; // Signal completion
            }
            break;
    }

    return false; // Still running
}