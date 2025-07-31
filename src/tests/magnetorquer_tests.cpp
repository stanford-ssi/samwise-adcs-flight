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

#define PWM_TEST_CURRENT_LIMIT (384)

static void test_magnetorquer_axis(const char *axis_name, int8_t x_duty,
                                   int8_t y_duty, int8_t z_duty,
                                   int test_time_ms)
{
    LOG_INFO("[mag_test] Testing %s magnetorquer at 100%% duty cycle",
             axis_name);

    pwm_error_t result =
        do_magnetorquer_pwm(x_duty, y_duty, z_duty, PWM_TEST_CURRENT_LIMIT);

    if (result == PWM_OK)
    {
        LOG_INFO("[mag_test] %s magnetorquer PWM set to 100%% (value: 128)",
                 axis_name);
        sleep_ms(test_time_ms);
    }
    else
    {
        LOG_ERROR("[mag_test] %s magnetorquer PWM failed with error: %d",
                  axis_name, result);
    }

    stop_magnetorquer_pwm();
    LOG_INFO("[mag_test] %s magnetorquer test complete", axis_name);
}

void magnetorquer_tests_init(void)
{
    LOG_INFO("[mag_test] Initializing magnetorquer PWM tests");
    init_magnetorquer_pwm();
}

bool magnetorquer_tests_dispatch(void)
{
    LOG_INFO("[mag_test] Running magnetorquer axis tests");

    int test_time = 5000; // 5 seconds for each axis

    // Test each axis at 100% duty cycle
    test_magnetorquer_axis("X", 128, 0, 0, test_time);
    test_magnetorquer_axis("Y", 0, 128, 0, test_time);
    test_magnetorquer_axis("Z", 0, 0, 128, test_time);

    LOG_INFO("[mag_test] All magnetorquer axis tests complete");
    return true;
}