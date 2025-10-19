/**
 * @author Lundeen Cahilly
 * @date 2025-08-19
 *
 * Frame transformation utilities for GNC algorithms,
 * provides conversions between different reference frames.
 */

#pragma once

#include "linalg.h"
using namespace linalg::aliases;

float3 enu_to_ecef(const float3 &enu, const float3 &lla);
float3 ecef_to_eci(const float3 &ecef, const float &MJD);
float3 eci_to_body(const float3 &eci, const quaternion &q_eci_to_body);
float3 body_to_eci(const float3 &body, const quaternion &q_eci_to_body);

#ifdef TEST
void test_transforms();
#endif