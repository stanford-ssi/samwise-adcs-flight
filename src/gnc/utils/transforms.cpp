/**
 * @author Lundeen Cahilly and Chen Li
 * @date 2025-08-19
 *
 * Frame transformation utilities for GNC algorithms,
 * provides conversions between different reference frames.
 */

#include "transforms.h"

#include "constants.h"
#include "gnc/utils/matrix_utils.h"
#include "gnc/utils/utils.h"
#include <cmath>

/**
 * Convert East-North-Up (ENU) coordinates to Earth-Centered Earth-Fixed (ECEF).
 *
 * @param enu Vector containing [East, North, Up] components (same units as
 * output)
 * @param lla Geodetic coordinates [lat_deg, lon_deg, (don't care)]
 * @return float3 Vector in ECEF coordinates (same units as input)
 */
float3 enu_to_ecef(const float3 &enu, const float3 &lla)
{
    const float lat = lla[0] * DEG_TO_RAD; // Latitude in radians (was lla[1])
    const float lon = lla[1] * DEG_TO_RAD; // Longitude in radians (was lla[2])

    // Rotation matrix from ENU to ECEF
    const float sin_lat = sinf(lat);
    const float cos_lat = cosf(lat);
    const float sin_lon = sinf(lon);
    const float cos_lon = cosf(lon);

    return {
        -sin_lon * enu[0] + (-sin_lat * cos_lon) * enu[1] +
            (cos_lat * cos_lon) * enu[2], // ECEF X
        cos_lon * enu[0] + (-sin_lat * sin_lon) * enu[1] +
            (cos_lat * sin_lon) * enu[2],          // ECEF Y
        0.0f + cos_lat * enu[1] + sin_lat * enu[2] // ECEF Z
    };
}

/**
 * Convert Earth-Centered Earth-Fixed (ECEF) coordinates to Earth-Centered
 * Inertial (ECI).
 *
 * @param enu Vector containing ECEF X Y Z components (same units as
 * output)
 * @param MJD Modified Julian Date
 * @return float3 Vector in ECI coordinates (same units as input)
 */

float3 ecef_to_eci(const float3 &ecef, const float &MJD)
{
    const float GMST = wrapTo360(280.4606 + 360.9856473 * (MJD - 51544.5)) *
                       DEG_TO_RAD; // Greenwich mean sidereal time in radians
    const float sin_GMST = sinf(GMST);
    const float cos_GMST = cosf(GMST);

    return {
        cos_GMST * ecef[0] - sin_GMST * ecef[1], // ECI I
        sin_GMST * ecef[0] + cos_GMST * ecef[1], // ECI J
        ecef[2]                                  // ECI K
    };
}