/**
 * @author Chen Li
 * @date 2025-09-05
 *
 * This file calculates Modified Julian Date based on GPS time, difference
 * between UTC and UT1 is ignored
 */

#pragma once

#include "linalg.h"

using namespace linalg::aliases;

float compute_MJD(float3 UTC_date, float gps_time);
