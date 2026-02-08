#pragma once

#include "params.h"
#include "linalg.h"
#include "slate.h"

using namespace linalg::aliases;

// Function declarations
float3 compute_control_torque(float4 q_current, float4 q_desired,
                              float3 omega_current, float3 omega_desired);

float4 allocate_reaction_wheels(float4 q_current, float4 q_desired,
                                float3 omega_current, float3 omega_desired);

#ifdef TEST
void test_reaction_wheel_allocation();
#endif