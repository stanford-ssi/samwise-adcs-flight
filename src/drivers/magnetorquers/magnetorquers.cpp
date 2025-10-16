/**
 * @author Lundeen Cahilly and Iris Xu
 * @date 2025-07-31
 *
 * Runs magnetorquers using PWM
 *
 */

#include "magnetorquers.h"

#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "macros.h"
#include "pins.h"
#include "slate.h"

// PWM Configuration Constants
#define PWM_WRAP_VALUE (128)
#define PWM_CLOCK_DIV (15625)
#define PWM_MAX_DUTY_CYCLE (127)
#define PWM_MIN_DUTY_CYCLE (-128)
#define PWM_DEFAULT_MAX_CURRENT (384)

/**
 * @brief Initialize PWM for magnetorquers
 *
 * This function configures the GPIO pins for PWM operation on all three
 * magnetorquer axes (X, Y, Z). Each axis has two PWM outputs (bidirectional
 * control)
 *
 */
void init_magnetorquer_pwm()
{
    // Enable pin
    gpio_init(SAMWISE_ADCS_EN_MAGDRV);
    gpio_set_dir(SAMWISE_ADCS_EN_MAGDRV, GPIO_OUT);
    gpio_put(SAMWISE_ADCS_EN_MAGDRV, 1); // Enable magnetorquers

    // Initialize pins
    gpio_init(SAMWISE_ADCS_X_MAGDRV_IN1);
    gpio_set_dir(SAMWISE_ADCS_X_MAGDRV_IN1, GPIO_OUT);
    gpio_init(SAMWISE_ADCS_X_MAGDRV_IN2);
    gpio_set_dir(SAMWISE_ADCS_X_MAGDRV_IN2, GPIO_OUT);

    gpio_init(SAMWISE_ADCS_Y_MAGDRV_IN1);
    gpio_set_dir(SAMWISE_ADCS_Y_MAGDRV_IN1, GPIO_OUT);
    gpio_init(SAMWISE_ADCS_Y_MAGDRV_IN2);
    gpio_set_dir(SAMWISE_ADCS_Y_MAGDRV_IN2, GPIO_OUT);

    gpio_init(SAMWISE_ADCS_Z_MAGDRV_IN1);
    gpio_set_dir(SAMWISE_ADCS_Z_MAGDRV_IN1, GPIO_OUT);
    gpio_init(SAMWISE_ADCS_Z_MAGDRV_IN2);
    gpio_set_dir(SAMWISE_ADCS_Z_MAGDRV_IN2, GPIO_OUT);

    // Configure GPIO pins for PWM function
    gpio_set_function(SAMWISE_ADCS_X_MAGDRV_IN1, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_X_MAGDRV_IN2, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Y_MAGDRV_IN1, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Y_MAGDRV_IN2, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Z_MAGDRV_IN1, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Z_MAGDRV_IN2, GPIO_FUNC_PWM);

    LOG_INFO("[magnetorquer] Magnetorquer PWM initialized");
}

/**
 * Set PWM duty cycles for magnetorquer control
 *
 * @param float3 magnetorquer_moment [-1, 1] in principal axes frame
 * @param slate_t *slate Pointer to the slate structure for state management
 * @return true if success, false if error
 */
bool do_magnetorquer_pwm(float3 magnetorquer_moment)
{
    // Set magnetorquer PWM duty cycles
    int8_t xdn =
        static_cast<int8_t>(magnetorquer_moment[0] * PWM_MAX_DUTY_CYCLE);
    int8_t ydn =
        static_cast<int8_t>(magnetorquer_moment[1] * PWM_MAX_DUTY_CYCLE);
    int8_t zdn =
        static_cast<int8_t>(magnetorquer_moment[2] * PWM_MAX_DUTY_CYCLE);

    // Validate input ranges
    if ((xdn > PWM_MAX_DUTY_CYCLE || xdn < PWM_MIN_DUTY_CYCLE) ||
        (ydn > PWM_MAX_DUTY_CYCLE || ydn < PWM_MIN_DUTY_CYCLE) ||
        (zdn > PWM_MAX_DUTY_CYCLE || zdn < PWM_MIN_DUTY_CYCLE))
    {
        LOG_ERROR(
            "[magnetorquer] Duty cycle values out of range (-128 to 127)");
        return false;
    }

    // Check total current consumption
    int total_current = abs(xdn) + abs(ydn) + abs(zdn);
    if (total_current > PWM_DEFAULT_MAX_CURRENT)
    {
        LOG_ERROR("[magnetorquer] Total current (%d) exceeds maximum (%d)",
                  total_current, PWM_DEFAULT_MAX_CURRENT);
        return false;
    }

    // Get PWM slices for each axis
    uint slice_x = pwm_gpio_to_slice_num(SAMWISE_ADCS_X_MAGDRV_IN1);
    uint slice_y = pwm_gpio_to_slice_num(SAMWISE_ADCS_Y_MAGDRV_IN1);
    uint slice_z = pwm_gpio_to_slice_num(SAMWISE_ADCS_Z_MAGDRV_IN1);

    // Configure PWM parameters for each slice
    uint slices[] = {slice_x, slice_y, slice_z};
    for (int i = 0; i < 3; i++)
    {
        pwm_set_wrap(slices[i], PWM_WRAP_VALUE);
        pwm_set_clkdiv(slices[i], PWM_CLOCK_DIV);
    }

    // Set PWM levels for X-axis based on direction
    pwm_set_gpio_level(SAMWISE_ADCS_X_MAGDRV_IN1, xdn >= 0 ? xdn : 0);
    pwm_set_gpio_level(SAMWISE_ADCS_X_MAGDRV_IN2, xdn < 0 ? -xdn : 0);

    // Set PWM levels for Y-axis based on direction
    pwm_set_gpio_level(SAMWISE_ADCS_Y_MAGDRV_IN1, ydn >= 0 ? ydn : 0);
    pwm_set_gpio_level(SAMWISE_ADCS_Y_MAGDRV_IN2, ydn < 0 ? -ydn : 0);

    // Set PWM levels for Z-axis based on direction
    pwm_set_gpio_level(SAMWISE_ADCS_Z_MAGDRV_IN1, zdn >= 0 ? zdn : 0);
    pwm_set_gpio_level(SAMWISE_ADCS_Z_MAGDRV_IN2, zdn < 0 ? -zdn : 0);

    // Enable PWM slices
    for (int i = 0; i < 3; i++)
    {
        pwm_set_enabled(slices[i], true);
    }

    // Return true to indicate success (and write to slate)
    return true;
}

/**
 * @brief Stop all magnetorquer PWM outputs
 *
 * This function disables all PWM outputs and sets duty cycles to zero
 * for safe shutdown of the magnetorquer system.
 */
void stop_magnetorquer_pwm(void)
{
    // Set all PWM outputs to zero
    pwm_set_gpio_level(SAMWISE_ADCS_X_MAGDRV_IN1, 0);
    pwm_set_gpio_level(SAMWISE_ADCS_X_MAGDRV_IN2, 0);
    pwm_set_gpio_level(SAMWISE_ADCS_Y_MAGDRV_IN1, 0);
    pwm_set_gpio_level(SAMWISE_ADCS_Y_MAGDRV_IN2, 0);
    pwm_set_gpio_level(SAMWISE_ADCS_Z_MAGDRV_IN1, 0);
    pwm_set_gpio_level(SAMWISE_ADCS_Z_MAGDRV_IN2, 0);

    // Disable PWM slices
    uint slice_x = pwm_gpio_to_slice_num(SAMWISE_ADCS_X_MAGDRV_IN1);
    uint slice_y = pwm_gpio_to_slice_num(SAMWISE_ADCS_Y_MAGDRV_IN1);
    uint slice_z = pwm_gpio_to_slice_num(SAMWISE_ADCS_Z_MAGDRV_IN1);

    pwm_set_enabled(slice_x, false);
    pwm_set_enabled(slice_y, false);
    pwm_set_enabled(slice_z, false);

    LOG_DEBUG("[magnetorquer] All magnetorquer outputs stopped");
}

#ifdef TEST
void test_magnetorquer_axis(const char *axis_name, float3 magnetorquer_moment,
                            int test_time_ms)
{
    LOG_INFO("[mag_test] Testing %s magnetorquer at duty: %d,%d,%d", axis_name,
             magnetorquer_moment.x, magnetorquer_moment.y,
             magnetorquer_moment.z);

    bool result = do_magnetorquer_pwm(magnetorquer_moment);

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
#endif