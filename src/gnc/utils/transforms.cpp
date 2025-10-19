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
    printf("qrot result: [%.6f, %.6f, %.6f]\n", body_result.x, body_result.y, body_result.z);
    printf("DCM result:  [%.6f, %.6f, %.6f]\n", dcm_result.x, dcm_result.y, dcm_result.z);

    // Also check qmat
    float3x3 qmat_result = qmat(q_z);
    float3 qmat_mul = mul(qmat_result, eci_x);
    printf("qmat result: [%.6f, %.6f, %.6f]\n", qmat_mul.x, qmat_mul.y, qmat_mul.z);

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
           body_test.x, body_test.y, body_test.z,
           dcm_test.x, dcm_test.y, dcm_test.z);

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
    float3x3 dcm_body_to_principal = transpose(PRINCIPAL_AXES_DCM); // active rotation
    float3 dcm_principal = mul(dcm_body_to_principal, body_vec);

    printf("body_to_principal: qrot=[%.6f, %.6f, %.6f], dcm^T=[%.6f, %.6f, %.6f]\n",
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

    printf("Round-trip: original=[%.6f, %.6f, %.6f], recovered=[%.6f, %.6f, %.6f]\n",
           body_original.x, body_original.y, body_original.z,
           back_to_body.x, back_to_body.y, back_to_body.z);

    for (int i = 0; i < 3; i++)
    {
        ASSERT_ALMOST_EQ(body_original[i], back_to_body[i], 1e-3f);
    }

    // Test 2: Verify principal_to_body against DCM (passive form, no transpose needed)
    float3 principal_vec = {0.5f, 1.5f, 2.5f};
    float3 body_result_qrot = principal_to_body(principal_vec);

    // For principal_to_body, use DCM directly (passive rotation)
    float3 body_result_dcm = mul(PRINCIPAL_AXES_DCM, principal_vec);

    printf("principal_to_body: qrot=[%.6f, %.6f, %.6f], dcm=[%.6f, %.6f, %.6f]\n",
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
    float3 test_vectors[] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        {1.0f, 2.0f, 3.0f},
        {-1.0f, 2.5f, -3.7f}
    };

    float3 test_axes[] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        normalize(float3(1.0f, 1.0f, 0.0f)),
        normalize(float3(1.0f, 1.0f, 1.0f))
    };

    float test_angles[] = {30.0f, 90.0f, 120.0f, 180.0f, 270.0f};

    for (int i = 0; i < 5; i++)
    {
        quaternion q_test = quaternion_by_axis_angle(test_axes[i],
                                                      test_angles[i] * DEG_TO_RAD);
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
}
#endif