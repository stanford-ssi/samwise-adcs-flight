/**
 * @author Lundeen Cahilly
 * @date 2025-07-14
 *
 * This file contains functions for controlling magnetorquers using PWM
 */
#pragma once

#include <stdint.h>

// Error codes
typedef enum
{
    PWM_OK = 0,
    PWM_ERROR_OUT_OF_RANGE,
    PWM_ERROR_CURRENT_EXCEEDED,
    PWM_ERROR_INVALID_PARAM
} pwm_error_t;

// PWM Configuration Constants
#define PWM_MAX_DUTY_CYCLE (128)
#define PWM_MIN_DUTY_CYCLE (-128)
#define PWM_DEFAULT_MAX_CURRENT (300)

// Initialize PWM for magnetorquer control
void init_pwm(void);

/**
 * Set PWM duty cycles for magnetorquer control
 *
 * @param xdn X-axis duty cycle (-128 to 128)
 * @param ydn Y-axis duty cycle (-128 to 128)
 * @param zdn Z-axis duty cycle (-128 to 128)
 * @param max_current Maximum allowed total current (sum of absolute values)
 * @return pwm_error_t Error code (PWM_OK on success)
 */
pwm_error_t do_pwm(int8_t xdn, int8_t ydn, int8_t zdn, int max_current);

/**
 * Stop all magnetorquer PWM outputs
 */
void stop_pwm(void);