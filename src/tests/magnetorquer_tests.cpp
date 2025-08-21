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

static void test_magnetorquer_axis(const char *axis_name,
                                   float3 magdrv_requested, int test_time_ms)
{
    LOG_INFO("[mag_test] Testing %s magnetorquer at duty: %d,%d,%d", axis_name,
             magdrv_requested.x, magdrv_requested.y, magdrv_requested.z);

    bool result = do_magnetorquer_pwm(magdrv_requested);

    if (!result)
    {
        LOG_ERROR("[mag_test] %s magnetorquer PWM failed", axis_name);
    }

    sleep_ms(test_time_ms);

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
    test_magnetorquer_axis("X", float3(127, 0, 0), test_time);
    test_magnetorquer_axis("X", float3(-128, 0, 0), test_time);
    test_magnetorquer_axis("Y", float3(0, 127, 0), test_time);
    test_magnetorquer_axis("Y", float3(0, -128, 0), test_time);
    test_magnetorquer_axis("Z", float3(0, 0, 127), test_time);
    test_magnetorquer_axis("Z", float3(0, 0, -128), test_time);

    LOG_INFO("[mag_test] All magnetorquer axis tests complete");
    return true;
}