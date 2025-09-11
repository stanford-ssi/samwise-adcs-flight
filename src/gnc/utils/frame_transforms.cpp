/**
 * @author Lundeen Cahilly
 * @date 2025-08-19
 *
 * Frame transformation utilities for GNC algorithms,
 * provides conversions between different reference frames.
 */

#include "frame_transforms.h"

#include "constants.h"
#include "gnc/utils/matrix_utils.h"
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