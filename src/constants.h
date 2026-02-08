/**
 * @author The ADCS team
 * @date 2025-02-10
 * Definition of all constants used in GNC
 */

#include "linalg.h"
using namespace linalg::aliases;

#pragma once

// ========================================================================
//          EARTH PARAMETERS
// ========================================================================

constexpr float R_E = 6378.0f; // Earth's equitorial radius [km]
constexpr float MU_EARTH =
    398600.4418f; // Earth gravitational parameter [km^3/s^2]

// WGS84 ellipsoid parameters
constexpr float a_EARTH = 6378.137f;             // Semi-major axis [km]
constexpr float f_EARTH = 1.0f / 298.257223563f; // Flattening
constexpr float E2_EARTH =
    2.0f * f_EARTH - f_EARTH * f_EARTH; // First eccentricity squared

// ========================================================================
//          UNIT CONVERSIONS
// ========================================================================

constexpr float DEG_TO_RAD = 0.01745329251;
constexpr float RAD_TO_DEG = 57.2957795131;
constexpr float KNOTS_TO_KMS =
    0.000514444f; // 1 knot = 1.852 km/h = 0.000514444 km/s

// ========================================================================
//          MATH
// ========================================================================

constexpr float SQRT_2_INV = 0.7071067811865476f; // 1 / sqrt(2)

constexpr float3x3 identity3x3 = {
    {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

constexpr float identity6x6[6 * 6] = {
    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
};
