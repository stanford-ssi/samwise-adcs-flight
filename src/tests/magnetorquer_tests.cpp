/**
 * @author  Lundeen Cahilly
 * @date    2025-07-17
 *
 * Test functions for magnetorquer pwm driver validation
 */

#include "magnetorquer_tests.h"
#include "../drivers/magnetorquer.h"
#include "macros.h"
#include "pico/stdlib.h"

// Test state tracking
typedef enum
{
    TEST_IDLE,
    TEST_4_1_BOOT,
    TEST_4_2_PWM_0_PERCENT,
    TEST_4_2_PWM_50_PERCENT,
    TEST_4_2_PWM_100_PERCENT,
    TEST_COMPLETE
} test_state_t;

static test_state_t current_test_state = TEST_IDLE;
static uint32_t test_start_time = 0;
static bool tests_complete = false;

// Test configuration
#define TEST_DURATION_MS (5000) // 5 seconds per test for manual observation
#define PWM_TEST_CURRENT_LIMIT (200)

/**
 * Test 4.1: Magnetorquer Boot Test
 * Enable magnetorquers, check that PWM system initializes properly
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
        LOG_INFO("[mag_test] 4.1 - PWM running at low duty cycle [10, 10, 10]");
        LOG_INFO(
            "[mag_test] 4.1 - Check voltage across magnetorquers manually");
        LOG_INFO(
            "[mag_test] 4.1 - Check current measurement (CM) data via mux");

        // Keep PWM running for manual measurement
        sleep_ms(3000);
    }
    else
    {
        LOG_ERROR("[mag_test] 4.1 - PWM initialization FAILED with error: %d",
                  result);
    }

    // Stop PWM after test
    stop_pwm();
    LOG_INFO("[mag_test] 4.1 - PWM stopped");
}

/**
 * Test 4.2: PWM Duty Cycle Test
 * Test magnetorquers at 0%, 50%, and 100% duty cycle
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

    if (duty_percent == 0)
    {
        LOG_INFO("[mag_test] 4.2 - Setting all PWM outputs to 0%% (OFF)");
        stop_pwm();
        sleep_ms(3000); // Give time to observe
        return;
    }

    // Apply PWM to X-axis only for this test
    pwm_error_t pwm_result = do_pwm(duty_value, 0, 0, PWM_TEST_CURRENT_LIMIT);

    if (pwm_result != PWM_OK)
    {
        LOG_ERROR("[mag_test] 4.2 - PWM setup failed with error: %d",
                  pwm_result);
        return;
    }

    LOG_INFO("[mag_test] 4.2 - X-axis PWM set to %d%% duty cycle (value: %d)",
             duty_percent, duty_value);
    LOG_INFO("[mag_test] 4.2 - Y and Z axes set to 0%%");
    LOG_INFO("[mag_test] 4.2 - Measure voltage/current on X-axis magnetorquer");

    // Keep PWM running for manual measurement
    sleep_ms(3000);

    // Stop PWM
    stop_pwm();
    LOG_INFO("[mag_test] 4.2 - PWM stopped");
}

void magnetorquer_tests_init(void)
{
    LOG_INFO("[mag_test] Initializing magnetorquer test module (PWM only)");

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
            LOG_INFO("[mag_test] Starting magnetorquer PWM test sequence");
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
                current_test_state = TEST_COMPLETE;
                test_start_time = current_time;
            }
            break;

        case TEST_COMPLETE:
            if (current_time - test_start_time >= TEST_DURATION_MS)
            {
                LOG_INFO("[mag_test] ===== MAGNETORQUER PWM TEST SEQUENCE "
                         "COMPLETE =====");
                LOG_INFO("[mag_test] Manual verification completed for:");
                LOG_INFO("[mag_test] - PWM initialization and basic operation");
                LOG_INFO(
                    "[mag_test] - 0%%, 50%%, and 100%% duty cycle operation");
                LOG_INFO(
                    "[mag_test] - Voltage measurements across magnetorquers");
                LOG_INFO("[mag_test] - Current measurement (CM) data via mux");
                tests_complete = true;
                return true; // Signal completion
            }
            break;
    }

    return false; // Still running
}