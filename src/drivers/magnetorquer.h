/**
 * @author Lundeen Cahilly and Iris Xu
 * @date 2025-07-31
 *
 * This file contains functions for controlling magnetorquers using PWM
 */

#pragma once

#include "linalg.h"
using namespace linalg::aliases;

// Initialize PWM for magnetorquer control
void init_magnetorquer_pwm(void);

/**
 * Set PWM duty cycles for magnetorquer control
 *
 * @param float3 magdrv_requested [-1, 1] in principal axes frame
 * @return true if success, false if error
 */
bool do_magnetorquer_pwm(float3 magdrv_requested);

/**
 * Stop all magnetorquer PWM outputs
 */
void stop_magnetorquer_pwm(void);