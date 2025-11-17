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

float3 lla_to_ecef(const float lat, const float lon, const float alt);
float3 speed_course_to_enu_velocity(const float speed, const float course);
float3 enu_to_ecef(const float3 &enu, const float3 &lla);
float3 ecef_to_eci(const float3 &ecef, const float &MJD);
float3 eci_to_body(const float3 &eci, const quaternion &q_eci_to_body);
float3 body_to_eci(const float3 &body, const quaternion &q_eci_to_body);
float3 body_to_principal(const float3 &body);
float3 principal_to_body(const float3 &principal);

#ifdef TEST
void test_transforms();
#endif