/**
 * @author Lundeen Cahilly and Iris Xu
 * @date 2025-07-31
 *
 * This file contains functions for controlling magnetorquers using PWM
 */

#pragma once

#include "linalg.h"

using namespace linalg::aliases;

struct Magnetorquer {
    float3 magtorq_requested;
    float3 magtorq_moment;
    float3 magtorq_duty_cycle;
    bool magnetorquers_running;
};

void init_magnetorquer_pwm(void);
bool do_magnetorquer_pwm(float3 magnetorquer_moment);
void stop_magnetorquer_pwm(void);

#ifdef TEST
void magnetorquer_tests_init(void);
bool magnetorquer_tests_dispatch(void);
#endif
