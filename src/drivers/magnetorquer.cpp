/**
 * @author Iris Xu and Lundeen Cahilly
 * @date 2025-07-14
 *
 * Runs magnetorquers using PWM
 *
 */

#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>

#include "macros.h"
#include "pins.h"

// PWM Configuration Constants
#define PWM_WRAP_VALUE (255)
#define PWM_CLOCK_DIV (15625)
#define PWM_MAX_DUTY_CYCLE (128)
#define PWM_MIN_DUTY_CYCLE (-128)

// Error codes
#define PWM_OK (0)
#define PWM_ERROR_OUT_OF_RANGE (1)
#define PWM_ERROR_CURRENT_EXCEEDED (2)

/**
 * Initialize PWM for magnetorquers
 *
 * This function configures the GPIO pins for PWM operation on all three
 * magnetorquer axes (X, Y, Z). Each axis has two PWM outputs (bidirectional
 * control)
 */
void init_pwm()
{
    // Configure GPIO pins for PWM function
    gpio_set_function(SAMWISE_ADCS_X_MAGDRV_IN1, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_X_MAGDRV_IN2, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Y_MAGDRV_IN1, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Y_MAGDRV_IN2, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Z_MAGDRV_IN1, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Z_MAGDRV_IN2, GPIO_FUNC_PWM);

    LOG_INFO("PWM: Magnetorquer PWM initialized");
}

/**
 * Set PWM duty cycles for magnetorquer control
 *
 * This function sets the PWM duty cycles for all three magnetorquer axes.
 * Positive values drive current in one direction, negative values in the
 * opposite direction. Uses current limiting.
 *
 * @param xdn x-axis duty cycle (-128 to 128)
 * @param ydn y-axis duty cycle (-128 to 128)
 * @param zdn z-axis duty cycle (-128 to 128)
 * @param max_current maximum allowed total current (sum of absolute values)
 * @return uint8_t error code (PWM_OK on success)
 */
uint8_t do_pwm(int8_t xdn, int8_t ydn, int8_t zdn, int max_current)
{
    // Validate input ranges
    if ((xdn > PWM_MAX_DUTY_CYCLE || xdn < PWM_MIN_DUTY_CYCLE) ||
        (ydn > PWM_MAX_DUTY_CYCLE || ydn < PWM_MIN_DUTY_CYCLE) ||
        (zdn > PWM_MAX_DUTY_CYCLE || zdn < PWM_MIN_DUTY_CYCLE))
    {
        LOG_ERROR("PWM: Duty cycle values out of range (-128 to 128)");
        return PWM_ERROR_OUT_OF_RANGE;
    }

    // Check total current consumption
    int total_current = abs(xdn) + abs(ydn) + abs(zdn);
    if (total_current > max_current)
    {
        LOG_ERROR("PWM: Total current (%d) exceeds maximum (%d)", total_current,
                  max_current);
        return PWM_ERROR_CURRENT_EXCEEDED;
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
    if (xdn >= 0)
    {
        pwm_set_gpio_level(SAMWISE_ADCS_X_MAGDRV_IN1, xdn);
        pwm_set_gpio_level(SAMWISE_ADCS_X_MAGDRV_IN2, 0);
    }
    else
    {
        pwm_set_gpio_level(SAMWISE_ADCS_X_MAGDRV_IN1, 0);
        pwm_set_gpio_level(SAMWISE_ADCS_X_MAGDRV_IN2, -xdn);
    }

    // Set PWM levels for Y-axis based on direction
    if (ydn >= 0)
    {
        pwm_set_gpio_level(SAMWISE_ADCS_Y_MAGDRV_IN1, ydn);
        pwm_set_gpio_level(SAMWISE_ADCS_Y_MAGDRV_IN2, 0);
    }
    else
    {
        pwm_set_gpio_level(SAMWISE_ADCS_Y_MAGDRV_IN1, 0);
        pwm_set_gpio_level(SAMWISE_ADCS_Y_MAGDRV_IN2, -ydn);
    }

    // Set PWM levels for Z-axis based on direction
    if (zdn >= 0)
    {
        pwm_set_gpio_level(SAMWISE_ADCS_Z_MAGDRV_IN1, zdn);
        pwm_set_gpio_level(SAMWISE_ADCS_Z_MAGDRV_IN2, 0);
    }
    else
    {
        pwm_set_gpio_level(SAMWISE_ADCS_Z_MAGDRV_IN1, 0);
        pwm_set_gpio_level(SAMWISE_ADCS_Z_MAGDRV_IN2, -zdn);
    }

    // Enable PWM slices
    for (int i = 0; i < 3; i++)
    {
        pwm_set_enabled(slices[i], true);
    }

    return PWM_OK;
}

/**
 * Stop all magnetorquer PWM outputs
 *
 * This function disables all PWM outputs and sets duty cycles to zero
 * for safe shutdown of the magnetorquer system.
 */
void stop_pwm()
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

    LOG_INFO("PWM: All magnetorquer outputs stopped");
}