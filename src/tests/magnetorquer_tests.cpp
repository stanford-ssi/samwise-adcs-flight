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

#define PWM_TEST_CURRENT_LIMIT (200)

static void test_pwm_duty_cycle(uint8_t duty_percent)
{
    LOG_INFO("[mag_test] Testing PWM at %d%% duty cycle", duty_percent);

    int8_t duty_value = 0;
    if (duty_percent == 50)
        duty_value = 64;
    else if (duty_percent == 100)
        duty_value = 128;

    if (duty_percent == 0)
    {
        LOG_INFO("[mag_test] Setting PWM to 0%% (OFF)");
        stop_pwm();
        return;
    }

    pwm_error_t result = do_pwm(duty_value, 0, 0, PWM_TEST_CURRENT_LIMIT);

    if (result == PWM_OK)
    {
        LOG_INFO("[mag_test] PWM set to %d%% (value: %d)", duty_percent,
                 duty_value);
    }
    else
    {
        LOG_ERROR("[mag_test] PWM failed with error: %d", result);
    }
}

void magnetorquer_tests_init(void)
{
    LOG_INFO("[mag_test] Initializing magnetorquer PWM tests");
    init_pwm();
}

bool magnetorquer_tests_dispatch(void)
{
    LOG_INFO("[mag_test] Running PWM duty cycle tests");

    test_pwm_duty_cycle(0);
    test_pwm_duty_cycle(50);
    test_pwm_duty_cycle(100);

    LOG_INFO("[mag_test] PWM tests complete");
    return true;
}