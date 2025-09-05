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

/**
 * Convert East-North-Up (ENU) coordinates to Earth-Centered Earth-Fixed (ECEF).
 *
 * @param enu Vector containing [East, North, Up] components (same units as
 * output)
 * @param lla Geodetic coordinates [lat_deg, lon_deg, (don't care)]
 * @return float3 Vector in ECEF coordinates (same units as input)
 */
float3 enu_to_ecef(const float3 &enu, const float3 &lla);

/**
 * Convert Earth-Centered Earth-Fixed (ECEF) coordinates to Earth-Centered
 * Inertial (ECI).
 *
 * @param enu Vector containing ECEF X Y Z components (same units as
 * output)
 * @param MJD Modified Julian Date
 * @return float3 Vector in ECI coordinates (same units as input)
 */

float3 ecef_to_eci(const float3 &ecef, const float &MJD);