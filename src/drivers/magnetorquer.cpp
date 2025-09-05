/**
 * @author Lundeen Cahilly and Iris Xu
 * @date 2025-07-31
 *
 * Runs magnetorquers using PWM
 *
 */

#include "magnetorquer.h"

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
 * Initialize PWM for magnetorquers
 *
 * This function configures the GPIO pins for PWM operation on all three
 * magnetorquer axes (X, Y, Z). Each axis has two PWM outputs (bidirectional
 * control)
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
 * @param float3 magdrv_requested [-1, 1] in principal axes frame
 * @param slate_t *slate Pointer to the slate structure for state management
 * @return true if success, false if error
 */
bool do_magnetorquer_pwm(float3 magdrv_requested)
{
    // Set magnetorquer PWM duty cycles
    int8_t xdn = static_cast<int8_t>(magdrv_requested[0] * PWM_MAX_DUTY_CYCLE);
    int8_t ydn = static_cast<int8_t>(magdrv_requested[1] * PWM_MAX_DUTY_CYCLE);
    int8_t zdn = static_cast<int8_t>(magdrv_requested[2] * PWM_MAX_DUTY_CYCLE);

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
 * Stop all magnetorquer PWM outputs
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