/**
 * @author The ADCS team
 * @date 2025-02-08
 */
#pragma once

#include "linalg.h"
#include "slate.h"

float3 bdot_compute_control_proportional(float3 dB, float dt);
float3 bdot_compute_control_bang_bang(float3 dB, float dt);

#ifdef TEST
void test_bdot_control(slate_t *slate);
#endif