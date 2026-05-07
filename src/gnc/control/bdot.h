/**
 * @author The ADCS team
 * @date 2025-02-08
 */
#pragma once

#include "linalg.h"
#include "pico/time.h"

using namespace linalg::aliases;

struct BDotController {
    float3 b_body_prev_;
    absolute_time_t b_body_read_time_prev_;
    bool bdot_has_prev_data_;
    bool bdot_data_has_updated_;
};

float3 bdot_compute_control_proportional(float3 dB, float dt);
float3 bdot_compute_control_bang_bang(float3 dB, float dt);

#ifdef TEST
void test_bdot_control(BDotController *bdot, MagnetometerData *bdot);
#endif
