#pragma once

#include "params.h"
#include "linalg.h"
#include "slate.h"

using namespace linalg::aliases;

bool saturation_detection(float3 &reaction_wheel_1, float3 &reaction_wheel_2,
                          float3 &reaction_wheel_3, float3 &reaction_wheel_4);

bool desaturation_detection(float3 &reaction_wheel_1, float3 &reaction_wheel_2,
                            float3 &reaction_wheel_3, float3 &reaction_wheel_4);

float3 compute_total_reaction_wheel_angular_momentum(float3 &reaction_wheel_1,
                                                     float3 &reaction_wheel_2,
                                                     float3 &reaction_wheel_3,
                                                     float3 &reaction_wheel_4);

float3
calculate_magnetometer_dipole(slate_t *slate,
                              float3 &total_reaction_wheel_angular_momentum);