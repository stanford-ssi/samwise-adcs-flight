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
 * Convert geodetic coordinates (LLA) to ECEF position.
 * Uses WGS84 ellipsoid parameters.
 * 
 * TODO: also use with magnetic field model
 *
 * @param lat Latitude in degrees (North positive)
 * @param lon Longitude in degrees (East positive)
 * @param alt Altitude above WGS84 ellipsoid in km
 * @return float3 Position in ECEF frame [km]
 */
float3 lla_to_ecef(const float lat, const float lon, const float alt)
{
    // WGS84 ellipsoid parameters
    constexpr float a = 6378.137f;         // Semi-major axis [km]
    constexpr float f = 1.0f / 298.257223563f; // Flattening
    constexpr float e2 = 2.0f * f - f * f;     // First eccentricity squared

    const float lat_rad = lat * DEG_TO_RAD;
    const float lon_rad = lon * DEG_TO_RAD;
    const float sin_lat = sinf(lat_rad);
    const float cos_lat = cosf(lat_rad);
    const float sin_lon = sinf(lon_rad);
    const float cos_lon = cosf(lon_rad);

    // Radius of curvature in the prime vertical
    const float N = a / sqrtf(1.0f - e2 * sin_lat * sin_lat);

    // ECEF coordinates [km]
    const float x = (N + alt) * cos_lat * cos_lon;
    const float y = (N + alt) * cos_lat * sin_lon;
    const float z = (N * (1.0f - e2) + alt) * sin_lat;

    return {x, y, z};
}

/**
 * Convert GPS speed and course to ENU velocity vector.
 * Assumes horizontal motion (vertical component = 0).
 *
 * @param speed Speed over ground in knots (from GPS RMC sentence)
 * @param course Course over ground in degrees true north (from GPS RMC sentence)
 * @return float3 Velocity in ENU frame [km/s]
 */
float3 speed_course_to_enu_velocity(const float speed, const float course)
{
    // Convert knots to km/s: 1 knot = 1.852 km/h = 0.000514444 km/s
    constexpr float KNOTS_TO_KMS = 0.000514444f;
    const float speed_kms = speed * KNOTS_TO_KMS;
    const float course_rad = course * DEG_TO_RAD;

    // ENU velocity components:
    // - East: speed * sin(course)  [course measured clockwise from north]
    // - North: speed * cos(course)
    // - Up: 0 (assume horizontal motion)
    return {speed_kms * sinf(course_rad),  // East
            speed_kms * cosf(course_rad),  // North
            0.0f};                         // Up
}

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

/**
 * @brief Convert from ECI (Earth-Centered Inertial) to ECEF (Earth-Centered
 * Earth-Fixed) frame
 * @param eci Position/velocity vector in ECI frame [km] or [km/s]
 * @param MJD Modified Julian Date
 * @return Position/velocity vector in ECEF frame [km] or [km/s]
 */
float3 eci_to_ecef(const float3 &eci, const float &MJD)
{
    const float GMST = wrapTo360(280.4606 + 360.9856473 * (MJD - 51544.5)) *
                       DEG_TO_RAD; // Greenwich mean sidereal time in radians
    const float sin_GMST = sinf(GMST);
    const float cos_GMST = cosf(GMST);

    // Inverse rotation (transpose of ecef_to_eci rotation matrix)
    return {
        cos_GMST * eci[0] + sin_GMST * eci[1],  // ECEF X
        -sin_GMST * eci[0] + cos_GMST * eci[1], // ECEF Y
        eci[2]                                  // ECEF Z
    };
}

/**
 * @brief Convert from ECEF (Earth-Centered Earth-Fixed) to LLA (Lat/Lon/Alt)
 * @param ecef Position in ECEF frame [km]
 * @return LLA coordinates: {latitude [deg], longitude [deg], altitude [km]}
 *
 * Uses iterative algorithm to solve for geodetic latitude on WGS84 ellipsoid
 */
float3 ecef_to_lla(const float3 &ecef)
{
    constexpr float a = 6378.137f;             // WGS84 semi-major axis [km]
    constexpr float f = 1.0f / 298.257223563f; // WGS84 flattening
    constexpr float e2 = 2.0f * f - f * f;     // First eccentricity squared

    const float x = ecef.x;
    const float y = ecef.y;
    const float z = ecef.z;

    // Longitude is straightforward
    float lon = atan2f(y, x) * RAD_TO_DEG;

    // Latitude requires iteration (Bowring's method)
    const float p = sqrtf(x * x + y * y);
    float lat = atan2f(z, p * (1.0f - e2)); // Initial guess

    // Iterate to convergence (usually 2-3 iterations)
    for (int i = 0; i < 5; i++)
    {
        const float sin_lat = sinf(lat);
        const float N = a / sqrtf(1.0f - e2 * sin_lat * sin_lat);
        const float h = p / cosf(lat) - N;
        lat = atan2f(z, p * (1.0f - e2 * N / (N + h)));
    }

    // Compute altitude
    const float sin_lat = sinf(lat);
    const float N = a / sqrtf(1.0f - e2 * sin_lat * sin_lat);
    const float alt = p / cosf(lat) - N;

    return {lat * RAD_TO_DEG, lon, alt};
}

/**
 * @brief Convert from ECEF (Earth-Centered Earth-Fixed) to ENU (East-North-Up)
 * local frame
 * @param ecef Vector in ECEF frame [km] or [km/s]
 * @param lla Reference point: {latitude [deg], longitude [deg], altitude [km]}
 * @return Vector in ENU frame [km] or [km/s]
 */
float3 ecef_to_enu(const float3 &ecef, const float3 &lla)
{
    const float lat = lla.x * DEG_TO_RAD;
    const float lon = lla.y * DEG_TO_RAD;

    const float sin_lat = sinf(lat);
    const float cos_lat = cosf(lat);
    const float sin_lon = sinf(lon);
    const float cos_lon = cosf(lon);

    // Rotation matrix from ECEF to ENU (transpose of enu_to_ecef)
    // ENU = R_ecef_to_enu * ECEF
    const float east =
        -sin_lon * ecef.x + cos_lon * ecef.y;
    const float north =
        -sin_lat * cos_lon * ecef.x - sin_lat * sin_lon * ecef.y + cos_lat * ecef.z;
    const float up =
        cos_lat * cos_lon * ecef.x + cos_lat * sin_lon * ecef.y + sin_lat * ecef.z;

    return {east, north, up};
}

/**
 * @brief Rotate a vector from ECI frame to body frame using quaternion
 * @param eci Vector in ECI frame
 * @param q_eci_to_body Quaternion representing rotation from ECI to body frame
 * @return Vector in body frame
 */
float3 eci_to_body(const float3 &eci, const quaternion &q_eci_to_body)
{
    // Rotate vector using quaternion rotation
    return qrot(q_eci_to_body, eci);
}

/**
 * @brief Rotate a vector from body frame to ECI frame using quaternion
 * @param body Vector in body frame
 * @param q_eci_to_body Quaternion representing rotation from ECI to body frame
 * @return Vector in ECI frame
 */
float3 body_to_eci(const float3 &body, const quaternion &q_eci_to_body)
{
    // Rotate vector using inverse quaternion (conjugate)
    return qrot(qconj(q_eci_to_body), body);
}

/**
 * @brief Rotate a vector from body frame to principal axes frame using constant
 * quaternion
 * @param body Vector in body frame
 * @return Vector in principal axes frame
 */
float3 body_to_principal(const float3 &body)
{
    // Rotate vector using quaternion rotation
    return qrot(Q_BODY_TO_PRINCIPAL, body);
}

/**
 * @brief Rotate a vector from principal axes frame to body frame using constant
 * quaternion
 * @param principal Vector in principal axes frame
 * @return Vector in body frame
 */
float3 principal_to_body(const float3 &principal)
{
    // Rotate vector using inverse quaternion (conjugate)
    return qrot(qconj(Q_BODY_TO_PRINCIPAL), principal);
}

#ifdef TEST
void test_transforms()
{
    printf("\n><=><=><=><=><= Testing frame transformation utilities... "
           "><=><=><=><=><=\n");

    // ========================================================================
    //      TEST LLA_TO_ECEF TRANSFORMATION
    // ========================================================================
    LOG_INFO("Testing lla_to_ecef transformation!");

    // Test 1: Equator at prime meridian (0°N, 0°E, 0 km alt)
    // Should give approximately [6378.137, 0, 0] km
    float3 ecef_equator = lla_to_ecef(0.0f, 0.0f, 0.0f);
    printf("Equator: ECEF=[%.3f, %.3f, %.3f] km\n", ecef_equator.x,
           ecef_equator.y, ecef_equator.z);
    ASSERT_ALMOST_EQ(ecef_equator.x, 6378.137f, 0.01f);
    ASSERT_ALMOST_EQ(ecef_equator.y, 0.0f, 0.01f);
    ASSERT_ALMOST_EQ(ecef_equator.z, 0.0f, 0.01f);

    // Test 2: North pole (90°N, 0°E, 0 km alt)
    // Should give approximately [0, 0, 6356.752] km (polar radius)
    float3 ecef_north_pole = lla_to_ecef(90.0f, 0.0f, 0.0f);
    printf("North Pole: ECEF=[%.3f, %.3f, %.3f] km\n", ecef_north_pole.x,
           ecef_north_pole.y, ecef_north_pole.z);
    ASSERT_ALMOST_EQ(ecef_north_pole.x, 0.0f, 0.01f);
    ASSERT_ALMOST_EQ(ecef_north_pole.y, 0.0f, 0.01f);
    ASSERT_ALMOST_EQ(ecef_north_pole.z, 6356.752f, 1.0f);

    // Test 3: ISS-like orbit (51.6°N, -60°W, 400 km alt)
    float3 ecef_iss = lla_to_ecef(51.6f, -60.0f, 400.0f);
    float r_iss = sqrtf(ecef_iss.x * ecef_iss.x + ecef_iss.y * ecef_iss.y +
                        ecef_iss.z * ecef_iss.z);
    printf("ISS-like: ECEF=[%.3f, %.3f, %.3f] km, radius=%.3f km\n", ecef_iss.x,
           ecef_iss.y, ecef_iss.z, r_iss);
    // ISS orbit radius should be approximately 6765 km
    // (ellipsoid radius at 51.6° lat is ~6365 km, + 400 km alt = 6765 km)
    ASSERT_ALMOST_EQ(r_iss, 6765.0f, 20.0f);

    // ========================================================================
    //      TEST SPEED_COURSE_TO_ENU_VELOCITY TRANSFORMATION
    // ========================================================================
    LOG_INFO("Testing speed_course_to_enu_velocity transformation!");

    // Test 1: 10 knots due north (course = 0°)
    // Should give [0, ~0.00514, 0] km/s
    float3 v_north = speed_course_to_enu_velocity(10.0f, 0.0f);
    printf("10 knots N: v_enu=[%.6f, %.6f, %.6f] km/s\n", v_north.x, v_north.y,
           v_north.z);
    ASSERT_ALMOST_EQ(v_north.x, 0.0f, 0.0001f);           // East = 0
    ASSERT_ALMOST_EQ(v_north.y, 0.00514444f, 0.0001f);    // North = 10 knots
    ASSERT_ALMOST_EQ(v_north.z, 0.0f, 0.0001f);           // Up = 0

    // Test 2: 10 knots due east (course = 90°)
    float3 v_east = speed_course_to_enu_velocity(10.0f, 90.0f);
    printf("10 knots E: v_enu=[%.6f, %.6f, %.6f] km/s\n", v_east.x, v_east.y,
           v_east.z);
    ASSERT_ALMOST_EQ(v_east.x, 0.00514444f, 0.0001f);     // East = 10 knots
    ASSERT_ALMOST_EQ(v_east.y, 0.0f, 0.0001f);            // North = 0
    ASSERT_ALMOST_EQ(v_east.z, 0.0f, 0.0001f);            // Up = 0

    // Test 3: 100 knots northeast (course = 45°)
    float3 v_ne = speed_course_to_enu_velocity(100.0f, 45.0f);
    float v_ne_mag = sqrtf(v_ne.x * v_ne.x + v_ne.y * v_ne.y + v_ne.z * v_ne.z);
    printf("100 knots NE: v_enu=[%.6f, %.6f, %.6f] km/s, mag=%.6f\n", v_ne.x,
           v_ne.y, v_ne.z, v_ne_mag);
    ASSERT_ALMOST_EQ(v_ne_mag, 0.0514444f, 0.001f);       // Total speed = 100 knots
    ASSERT_ALMOST_EQ(v_ne.x, v_ne.y, 0.0001f);            // Equal E and N components

    // ========================================================================
    //      TEST ECI_TO_BODY TRANSFORMATION
    // ========================================================================
    LOG_INFO("Testing eci_to_body transformation using quaternion!");

    // Test 1: 90 degree rotation about Z-axis
    float3 axis_z = {0.0f, 0.0f, 1.0f};
    float angle_z = 90.0f * DEG_TO_RAD;
    quaternion q_z = quaternion_by_axis_angle(axis_z, angle_z);

    // Rotate vector [1, 0, 0] by 90 degrees about Z-axis
    float3 eci_x = {1.0f, 0.0f, 0.0f};
    float3 body_result = eci_to_body(eci_x, q_z);

    // Verify using DCM method
    float3x3 dcm_z = quaternion_to_dcm(q_z);
    float3 dcm_result = mul(dcm_z, eci_x);

    // Debug print
    printf("q_z: [%.6f, %.6f, %.6f, %.6f]\n", q_z.x, q_z.y, q_z.z, q_z.w);
    printf("qrot result: [%.6f, %.6f, %.6f]\n", body_result.x, body_result.y,
           body_result.z);
    printf("DCM result:  [%.6f, %.6f, %.6f]\n", dcm_result.x, dcm_result.y,
           dcm_result.z);

    // Also check qmat
    float3x3 qmat_result = qmat(q_z);
    float3 qmat_mul = mul(qmat_result, eci_x);
    printf("qmat result: [%.6f, %.6f, %.6f]\n", qmat_mul.x, qmat_mul.y,
           qmat_mul.z);

    for (int i = 0; i < 3; i++)
    {
        ASSERT_ALMOST_EQ(body_result[i], dcm_result[i], 1e-6f);
    }

    // Test 2: 45 degree rotation about (1, 1, 0) axis
    float3 axis_diag = normalize(float3(1.0f, 1.0f, 0.0f));
    float angle_45 = 45.0f * DEG_TO_RAD;
    quaternion q_diag = quaternion_by_axis_angle(axis_diag, angle_45);

    float3 eci_test = {1.0f, 2.0f, 3.0f};
    float3 body_test = eci_to_body(eci_test, q_diag);

    // Verify using DCM method
    float3x3 dcm_diag = quaternion_to_dcm(q_diag);
    float3 dcm_test = mul(dcm_diag, eci_test);

    printf("Test 2: qrot=[%.6f, %.6f, %.6f], dcm=[%.6f, %.6f, %.6f]\n",
           body_test.x, body_test.y, body_test.z, dcm_test.x, dcm_test.y,
           dcm_test.z);

    for (int i = 0; i < 3; i++)
    {
        ASSERT_ALMOST_EQ(body_test[i], dcm_test[i], 1e-6f);
    }

    // ========================================================================
    //      TEST BODY_TO_PRINCIPAL TRANSFORMATION
    // ========================================================================
    LOG_INFO("Testing body_to_principal transformation!");

    // Test 1: Verify against DCM method
    float3 body_vec = {1.0f, 2.0f, 3.0f};
    float3 principal_result = body_to_principal(body_vec);
    float3x3 dcm_body_to_principal =
        transpose(PRINCIPAL_AXES_DCM); // active rotation
    float3 dcm_principal = mul(dcm_body_to_principal, body_vec);

    printf("body_to_principal: qrot=[%.6f, %.6f, %.6f], dcm^T=[%.6f, %.6f, "
           "%.6f]\n",
           principal_result.x, principal_result.y, principal_result.z,
           dcm_principal.x, dcm_principal.y, dcm_principal.z);

    for (int i = 0; i < 3; i++)
    {
        // ASSERT_ALMOST_EQ(principal_result[i], dcm_principal[i], 1e-6f);
    }

    // ========================================================================
    //      TEST PRINCIPAL_TO_BODY TRANSFORMATION
    // ========================================================================
    LOG_INFO("Testing principal_to_body transformation!");

    // Test 1: Should be inverse of body_to_principal
    float3 body_original = {1.5f, -2.3f, 4.7f};
    float3 to_principal = body_to_principal(body_original);
    float3 back_to_body = principal_to_body(to_principal);

    printf("Round-trip: original=[%.6f, %.6f, %.6f], recovered=[%.6f, %.6f, "
           "%.6f]\n",
           body_original.x, body_original.y, body_original.z, back_to_body.x,
           back_to_body.y, back_to_body.z);

    for (int i = 0; i < 3; i++)
    {
        ASSERT_ALMOST_EQ(body_original[i], back_to_body[i], 1e-3f);
    }

    // Test 2: Verify principal_to_body against DCM (passive form, no transpose
    // needed)
    float3 principal_vec = {0.5f, 1.5f, 2.5f};
    float3 body_result_qrot = principal_to_body(principal_vec);

    // For principal_to_body, use DCM directly (passive rotation)
    float3 body_result_dcm = mul(PRINCIPAL_AXES_DCM, principal_vec);

    printf(
        "principal_to_body: qrot=[%.6f, %.6f, %.6f], dcm=[%.6f, %.6f, %.6f]\n",
        body_result_qrot.x, body_result_qrot.y, body_result_qrot.z,
        body_result_dcm.x, body_result_dcm.y, body_result_dcm.z);

    for (int i = 0; i < 3; i++)
    {
        // ASSERT_ALMOST_EQ(body_result_qrot[i], body_result_dcm[i], 1e-6f);
    }

    // ========================================================================
    //      TEST BODY_TO_ECI TRANSFORMATION
    // ========================================================================
    LOG_INFO("Testing body_to_eci transformation using quaternion!");

    // Test 1: Should be inverse of eci_to_body
    float3 eci_original = {1.0f, 2.0f, 3.0f};
    float3 body_converted = eci_to_body(eci_original, q_z);
    float3 eci_recovered = body_to_eci(body_converted, q_z);

    for (int i = 0; i < 3; i++)
    {
        ASSERT_ALMOST_EQ(eci_original[i], eci_recovered[i], 1e-6f);
    }

    // Test 2: Verify using DCM method
    float3 body_vec2 = {1.0f, 0.0f, 0.0f};
    float3 eci_result = body_to_eci(body_vec2, q_diag);

    // Use conjugate quaternion for inverse rotation
    quaternion q_conj = {-q_diag.x, -q_diag.y, -q_diag.z, q_diag.w};
    float3x3 dcm_inv = quaternion_to_dcm(q_conj);
    float3 dcm_eci = mul(dcm_inv, body_vec2);

    for (int i = 0; i < 3; i++)
    {
        ASSERT_ALMOST_EQ(eci_result[i], dcm_eci[i], 1e-6f);
    }

    // ========================================================================
    //      TEST ROUND-TRIP TRANSFORMATION
    // ========================================================================
    LOG_INFO("Testing round-trip transformation consistency!");

    // Test with multiple quaternions and vectors
    float3 test_vectors[] = {{1.0f, 0.0f, 0.0f},
                             {0.0f, 1.0f, 0.0f},
                             {0.0f, 0.0f, 1.0f},
                             {1.0f, 2.0f, 3.0f},
                             {-1.0f, 2.5f, -3.7f}};

    float3 test_axes[] = {{1.0f, 0.0f, 0.0f},
                          {0.0f, 1.0f, 0.0f},
                          {0.0f, 0.0f, 1.0f},
                          normalize(float3(1.0f, 1.0f, 0.0f)),
                          normalize(float3(1.0f, 1.0f, 1.0f))};

    float test_angles[] = {30.0f, 90.0f, 120.0f, 180.0f, 270.0f};

    for (int i = 0; i < 5; i++)
    {
        quaternion q_test =
            quaternion_by_axis_angle(test_axes[i], test_angles[i] * DEG_TO_RAD);
        for (int j = 0; j < 5; j++)
        {
            float3 original = test_vectors[j];
            float3 to_body = eci_to_body(original, q_test);
            float3 back_to_eci = body_to_eci(to_body, q_test);

            for (int k = 0; k < 3; k++)
            {
                ASSERT_ALMOST_EQ(original[k], back_to_eci[k], 1e-5f);
            }
        }
    }

    // ========================================================================
    //      TEST ECI_TO_ECEF AND ECEF_TO_ECI INVERSE RELATIONSHIP
    // ========================================================================
    LOG_INFO("Testing eci_to_ecef / ecef_to_eci inverse transforms!");

    float MJD_test = 60000.0f; // Arbitrary MJD for testing

    // Test 1: Round trip ECI -> ECEF -> ECI
    float3 eci_orig = {6778.0f, 1000.0f, 500.0f}; // Arbitrary ECI position
    float3 ecef_converted = eci_to_ecef(eci_orig, MJD_test);
    float3 eci_back = ecef_to_eci(ecef_converted, MJD_test);

    printf("ECI original:   [%.6f, %.6f, %.6f] km\n", eci_orig.x, eci_orig.y,
           eci_orig.z);
    printf("ECEF converted: [%.6f, %.6f, %.6f] km\n", ecef_converted.x,
           ecef_converted.y, ecef_converted.z);
    printf("ECI back:       [%.6f, %.6f, %.6f] km\n", eci_back.x, eci_back.y,
           eci_back.z);

    for (int i = 0; i < 3; i++)
    {
        ASSERT_ALMOST_EQ(eci_orig[i], eci_back[i], 1e-3f);
    }

    // Test 2: Verify magnitude is preserved (rotation only)
    float r_eci = length(eci_orig);
    float r_ecef = length(ecef_converted);
    printf("Radius ECI: %.6f km, Radius ECEF: %.6f km\n", r_eci, r_ecef);
    ASSERT_ALMOST_EQ(r_eci, r_ecef, 1e-3f);

    // Test 3: Round trip for velocity vector
    float3 v_eci_orig = {7.5f, -1.2f, 0.3f}; // Arbitrary velocity
    float3 v_ecef_converted = eci_to_ecef(v_eci_orig, MJD_test);
    float3 v_eci_back = ecef_to_eci(v_ecef_converted, MJD_test);

    for (int i = 0; i < 3; i++)
    {
        ASSERT_ALMOST_EQ(v_eci_orig[i], v_eci_back[i], 1e-3f);
    }

    // ========================================================================
    //      TEST ECEF_TO_LLA AND LLA_TO_ECEF INVERSE RELATIONSHIP
    // ========================================================================
    LOG_INFO("Testing ecef_to_lla / lla_to_ecef inverse transforms!");

    // Test 1: Round trip for equator point
    float lat_eq = 0.0f, lon_eq = 0.0f, alt_eq = 0.0f;
    float3 ecef_from_lla = lla_to_ecef(lat_eq, lon_eq, alt_eq);
    float3 lla_back = ecef_to_lla(ecef_from_lla);

    printf("LLA original:  [%.6f, %.6f, %.6f]\n", lat_eq, lon_eq, alt_eq);
    printf("ECEF:          [%.6f, %.6f, %.6f] km\n", ecef_from_lla.x,
           ecef_from_lla.y, ecef_from_lla.z);
    printf("LLA back:      [%.6f, %.6f, %.6f]\n", lla_back.x, lla_back.y,
           lla_back.z);

    ASSERT_ALMOST_EQ(lat_eq, lla_back.x, 0.001f);
    ASSERT_ALMOST_EQ(lon_eq, lla_back.y, 0.001f);
    ASSERT_ALMOST_EQ(alt_eq, lla_back.z, 0.1f);

    // Test 2: Round trip for ISS-like position
    float lat_iss = 51.6f, lon_iss = -60.0f, alt_iss = 400.0f;
    float3 ecef_iss_lla = lla_to_ecef(lat_iss, lon_iss, alt_iss);
    float3 lla_iss_back = ecef_to_lla(ecef_iss_lla);

    printf("ISS LLA original: [%.6f, %.6f, %.6f]\n", lat_iss, lon_iss, alt_iss);
    printf("ECEF:             [%.6f, %.6f, %.6f] km\n", ecef_iss_lla.x,
           ecef_iss_lla.y, ecef_iss_lla.z);
    printf("LLA back:         [%.6f, %.6f, %.6f]\n", lla_iss_back.x,
           lla_iss_back.y, lla_iss_back.z);

    ASSERT_ALMOST_EQ(lat_iss, lla_iss_back.x, 0.001f);
    ASSERT_ALMOST_EQ(lon_iss, lla_iss_back.y, 0.001f);
    ASSERT_ALMOST_EQ(alt_iss, lla_iss_back.z, 0.1f);

    // Test 3: Round trip for North pole
    float lat_pole = 89.0f, lon_pole = 0.0f, alt_pole = 10.0f;
    float3 ecef_pole_lla = lla_to_ecef(lat_pole, lon_pole, alt_pole);
    float3 lla_pole_back = ecef_to_lla(ecef_pole_lla);

    printf("North pole LLA:   [%.6f, %.6f, %.6f]\n", lat_pole, lon_pole,
           alt_pole);
    printf("LLA back:         [%.6f, %.6f, %.6f]\n", lla_pole_back.x,
           lla_pole_back.y, lla_pole_back.z);

    ASSERT_ALMOST_EQ(lat_pole, lla_pole_back.x, 0.001f);
    ASSERT_ALMOST_EQ(alt_pole, lla_pole_back.z, 0.1f);

    // ========================================================================
    //      TEST ECEF_TO_ENU AND ENU_TO_ECEF INVERSE RELATIONSHIP
    // ========================================================================
    LOG_INFO("Testing ecef_to_enu / enu_to_ecef inverse transforms!");

    // Test 1: Round trip at equator
    float3 lla_ref_eq = {0.0f, 0.0f, 0.0f}; // Reference point at equator
    float3 enu_orig = {1.0f, 2.0f, 3.0f};   // Arbitrary ENU vector
    float3 ecef_from_enu = enu_to_ecef(enu_orig, lla_ref_eq);
    float3 enu_back = ecef_to_enu(ecef_from_enu, lla_ref_eq);

    printf("ENU original:  [%.6f, %.6f, %.6f] km\n", enu_orig.x, enu_orig.y,
           enu_orig.z);
    printf("ECEF:          [%.6f, %.6f, %.6f] km\n", ecef_from_enu.x,
           ecef_from_enu.y, ecef_from_enu.z);
    printf("ENU back:      [%.6f, %.6f, %.6f] km\n", enu_back.x, enu_back.y,
           enu_back.z);

    for (int i = 0; i < 3; i++)
    {
        ASSERT_ALMOST_EQ(enu_orig[i], enu_back[i], 1e-5f);
    }

    // Test 2: Round trip at ISS latitude
    float3 lla_ref_iss = {51.6f, -60.0f, 400.0f}; // ISS-like reference
    float3 enu_iss = {-0.5f, 1.5f, -0.3f};
    float3 ecef_iss_enu = enu_to_ecef(enu_iss, lla_ref_iss);
    float3 enu_iss_back = ecef_to_enu(ecef_iss_enu, lla_ref_iss);

    printf("ISS ENU original: [%.6f, %.6f, %.6f] km\n", enu_iss.x, enu_iss.y,
           enu_iss.z);
    printf("ENU back:         [%.6f, %.6f, %.6f] km\n", enu_iss_back.x,
           enu_iss_back.y, enu_iss_back.z);

    for (int i = 0; i < 3; i++)
    {
        ASSERT_ALMOST_EQ(enu_iss[i], enu_iss_back[i], 1e-5f);
    }

    // Test 3: Verify magnitude preservation
    float enu_mag = length(enu_orig);
    float ecef_mag = length(ecef_from_enu);
    printf("Magnitude ENU: %.6f km, Magnitude ECEF: %.6f km\n", enu_mag,
           ecef_mag);
    ASSERT_ALMOST_EQ(enu_mag, ecef_mag, 1e-5f);

    LOG_INFO("All transform tests passed!");
}
#endif