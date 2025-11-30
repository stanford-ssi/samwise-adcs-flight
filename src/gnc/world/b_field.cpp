/**
 * @author Lundeen Cahilly
 * @date 2025-02-10
 *
 * This file defines a magnetic field model based on the IGRF-14 2025
 * coefficients. It computes the magnetic field vector in based on the
 * satellite's geodetic coordinates (altitude, latitude, longitude).
 */

#include "b_field.h"
#include "b_field_constants.h"

#include "pico/stdlib.h"
#include <cmath>

#include "constants.h"
#include "gnc/utils/transforms.h"
#include "linalg.h"

// Forward declaration for legendre polynomial computation
void compute_legendre_polynomials(int nmax, float theta,
                                  float Pnm[][B_FIELD_MODEL_MAX_ORDER + 2],
                                  float cos_theta, float sin_theta);

/**
 * @brief Get precomputed square root value from lookup table, function makes
 * code look cleaner
 *
 * @param index Index into the lookup table
 * @return float Precomputed square root value
 */
float sqrt_lut_get(int index)
{
    return sqrt_lut[index];
}

/**
 * @brief Compute the magnetic field vector in ECEF and ECI frames based on
 * geodetic coordinates using the IGRF-14 model. Does NOT use time-varying
 * coefficients (TODO: add for next satellite).
 *
 * @param slate Pointer to the slate structure containing GPS coordinates
 * @return true if computation was successful, false if there was an error
 */
bool compute_B(slate_t *slate)
{
    const float lat = slate->gps_lat; // latitude (-90 to 90)
    const float lon = slate->gps_lon; // longitude (-180 to 180)
    const float alt = slate->gps_alt; // altitude (km)

    // Input validation to prevent NaN
    if (alt < B_FIELD_LOW_ALTITUDE_THRESH || alt > B_FIELD_HIGH_ALTITUDE_THRESH)
    {
        LOG_ERROR("Invalid altitude: %f km", alt);
        return false;
    }
    if (lat < -90.0f || lat > 90.0f)
    {
        LOG_ERROR("Invalid latitude: %f degrees", lat);
        return false;
    }
    if (lon < -180.0f || lon > 180.0f)
    {
        LOG_ERROR("Invalid longitude: %f degrees", lon);
        return false;
    }

    // Using math spherical coordinate conventions
    const float theta = (90.0f - lat) * DEG_TO_RAD; // colatitude [0 to π]
    const float phi = lon * DEG_TO_RAD;             // azimuth [-π to π]

    float b_r = 0.0f;
    float b_theta = 0.0f;
    float b_phi = 0.0f;

    const float r_ratio = R_E / (R_E + alt);

    // Cache trig terms with pole regularization
    const float sin_theta = sinf(theta); // for pole regularization
    const float cos_theta = cosf(theta); // for Legendre polynomials
    if (fabsf(sin_theta) < B_FIELD_POLE_THRESH)
    {
        LOG_ERROR("Invalid theta: %f radians (near pole singularity)", theta);
        return false; // Avoid singularities near poles
    }

    // Pre-compute sin/cos m*phi terms
    float sin_mphi[B_FIELD_MODEL_ORDER + 1];
    float cos_mphi[B_FIELD_MODEL_ORDER + 1];
    for (int m = 0; m <= B_FIELD_MODEL_ORDER; m++)
    {
        sin_mphi[m] = sinf(m * phi);
        cos_mphi[m] = cosf(m * phi);
    }

    // Compute legendre polynomials and derivatives
    float Pnm[B_FIELD_MODEL_MAX_ORDER + 1][B_FIELD_MODEL_MAX_ORDER + 2];
    compute_legendre_polynomials(B_FIELD_MODEL_ORDER, theta, Pnm, cos_theta,
                                 sin_theta);

    // Accumulate field components
    float r_ratio_n = r_ratio * r_ratio * r_ratio; // Start at n=1 term
    for (int n = 1; n <= B_FIELD_MODEL_ORDER; n++)
    {
        for (int m = 0; m <= n; m++)
        {
            // Get Schmidt normalized associated Legendre functions and
            // derivatives
            const float P = Pnm[n][m];
            const float dP = Pnm[m][n + 1]; // store derivatives at [m][n+1]

            // Compute common term for efficiency
            const float term = (g[n][m] * cos_mphi[m] + h[n][m] * sin_mphi[m]);

            // Accumulate field components
            // Magnetic field in r direction
            b_r += static_cast<float>(n + 1) * r_ratio_n * P * term;
            // Magnetic field in latitude direction
            b_theta -= r_ratio_n * dP * term;

            // Handle b_phi carefully near poles
            if (m > 0)
            { // m=0 terms don't contribute to b_phi
                const float b_phi_term =
                    (-g[n][m] * sin_mphi[m] + h[n][m] * cos_mphi[m]) *
                    static_cast<float>(m) * P / sin_theta;
                b_phi -= r_ratio_n * b_phi_term;
            }
        }
        r_ratio_n *= r_ratio; // Increment r_ratio^n for next n
    }

    // Check for NaN values before storing
    if (std::isnan(b_r) || std::isnan(b_theta) || std::isnan(b_phi) ||
        std::isinf(b_r) || std::isinf(b_theta) || std::isinf(b_phi))
    {
        LOG_ERROR(
            "NaN/Inf detected in magnetic field: b_r=%f, b_theta=%f, b_phi=%f",
            b_r, b_theta, b_phi);
        LOG_ERROR("Input coordinates: alt=%f, lat=%f, lon=%f", alt, lat, lon);
        return false;
    }

    // Store raw results (currently for debugging)
    // TODO: could be used for attitude-independent calibration
    float3 b_enu_raw = {b_phi, -b_theta, -b_r}; // East-North-Up (ENU) frame
    float3 b_ecef_raw = enu_to_ecef(
        b_enu_raw, float3(lat, lon, alt)); // Passive transform to ECEF frame
    float3 b_eci_raw = ecef_to_eci(
        b_ecef_raw, slate->MJD); // Passive transformation to ECI frame

    slate->b_ecef = normalize(b_ecef_raw);
    slate->b_eci = normalize(b_eci_raw);

#ifdef TEST
    // Define these fields only in test builds
    float3 b_rpt_raw = {b_r, b_phi, b_theta};
    slate->b_rpt = normalize(b_rpt_raw);
    slate->b_enu = normalize(b_enu_raw);
#endif

    return true;
}

/**
 * @brief Compute Schmidt semi-normalized associated Legendre polynomials
 *        and their derivatives using recursion relations. Based on pyigrf.
 *
 * @param nmax Maximum degree/order of the polynomials
 * @param theta Colatitude in radians (0 at North Pole, π at South Pole)
 * @param Pnm Output array to store polynomials P(n,m) and derivatives dP(n,m)
 *            Pnm[n][m] = P(n,m)
 *            Pnm[m][n+1] = dP(n,m)
 * @param cos_theta Precomputed cosine of theta
 * @param sin_theta Precomputed sine of theta
 */
void compute_legendre_polynomials(int nmax, float theta,
                                  float Pnm[][B_FIELD_MODEL_MAX_ORDER + 2],
                                  float cos_theta, float sin_theta)
{
    // Initialize arrays
    for (int n = 0; n <= nmax; n++)
    {
        for (int m = 0; m <= nmax + 1; m++)
        {
            Pnm[n][m] = 0.0f;
        }
    }

    // Base cases, use dynamic programming to fill in rest for efficiency
    Pnm[0][0] = 1.0f;      // P(0,0) = 1
    Pnm[1][1] = sin_theta; // P(1,1) = sin(theta)

    // Recursion relations after Langel "The Main Field" (1987)
    for (int m = 0; m < nmax; m++)
    {
        float Pnm_tmp = sqrt_lut_get(m + m + 1) * Pnm[m][m];
        Pnm[m + 1][m] = cos_theta * Pnm_tmp;

        if (m > 0)
        {
            Pnm[m + 1][m + 1] = sin_theta * Pnm_tmp / sqrt_lut_get(m + m + 2);
        }

        for (int n = m + 2; n <= nmax; n++)
        {
            int d = n * n - m * m;
            int e = n + n - 1;
            Pnm[n][m] = (static_cast<float>(e) * cos_theta * Pnm[n - 1][m] -
                         sqrt_lut_get(d - e) * Pnm[n - 2][m]) /
                        sqrt_lut_get(d);
        }
    }

    // Compute derivatives: dP(n,m) = Pnm[m][n+1]
    Pnm[0][2] = -Pnm[1][1];
    Pnm[1][2] = Pnm[1][0];

    for (int n = 2; n <= nmax; n++)
    {
        Pnm[0][n + 1] = -sqrt_lut_get(n * n + n) * SQRT_2_INV * Pnm[n][1];
        Pnm[1][n + 1] = (sqrt_lut_get(2 * (n * n + n)) * Pnm[n][0] -
                         sqrt_lut_get(n * n + n - 2) * Pnm[n][2]) /
                        2.0f;

        for (int m = 2; m < n; m++)
        {
            // For these products, we need to check if they're within our LUT
            // bounds
            int prod1 = (n + m) * (n - m + 1);
            int prod2 = (n + m + 1) * (n - m);
            Pnm[m][n + 1] = 0.5f * (sqrt_lut_get(prod1) * Pnm[n][m - 1] -
                                    sqrt_lut_get(prod2) * Pnm[n][m + 1]);
        }
        Pnm[n][n + 1] =
            sqrtf(2.0f * static_cast<float>(n)) * Pnm[n][n - 1] / 2.0f;
    }
}

#ifdef TEST
void test_b_field_reference_points(slate_t *slate)
{
    LOG_INFO("Testing B field reference points...");

    const float DOT_PRODUCT_THRESHOLD = 0.99f; // Directional accuracy threshold

    struct TestPoint
    {
        float lat, lon, alt;
        float expected_b_r, expected_b_phi, expected_b_theta, expected_mag;
        const char *name;
    };

    // Reference values from IGRF calculator at mean satellite altitude
    TestPoint test_points[] = {{89.9f, 0.0f, 0.0f, -56835.0f, 439.0f, -1783.0f,
                                56864.0f, "North Pole"},
                               {-89.9f, 0.0f, 0.0f, 51612.0f, -8771.0f,
                                -14391.0f, 54294.0f, "South Pole"},
                               {0.0f, 0.0f, 0.0f, 15997.0f, -1926.0f, -27457.0f,
                                31835.0f, "Equator at Greenwich"},
                               {45.0f, -75.0f, 300.0f, -42838.0f, -3417.0f,
                                -15976.0f, 45848.0f, "Mid-latitude point"},
                               {-30.0f, -45.0f, 0.0f, 16572.0f, -5499.0f,
                                -14603.0f, 22762.0f, "South Atlantic Anomaly"}};

    int num_tests = sizeof(test_points) / sizeof(TestPoint);
    int passed = 0;

    for (int i = 0; i < num_tests; i++)
    {
        slate->gps_lat = test_points[i].lat;
        slate->gps_lon = test_points[i].lon;
        slate->gps_alt = test_points[i].alt;

        compute_B(slate);

        // Normalize expected vector for direction comparison
        float3 expected_B = {test_points[i].expected_b_r,
                             test_points[i].expected_b_phi,
                             test_points[i].expected_b_theta};
        float expected_magnitude = sqrtf(expected_B[0] * expected_B[0] +
                                         expected_B[1] * expected_B[1] +
                                         expected_B[2] * expected_B[2]);
        expected_B = expected_B / expected_magnitude;

        // Compute dot product for directional comparison
        float dot_product = slate->b_rpt[0] * expected_B[0] +
                            slate->b_rpt[1] * expected_B[1] +
                            slate->b_rpt[2] * expected_B[2];

        bool test_passed = dot_product >= DOT_PRODUCT_THRESHOLD;

        LOG_INFO("%s: %s", test_points[i].name, test_passed ? "PASS" : "FAIL");
        LOG_INFO("  Computed: b_r=%.10f, b_phi=%.10f, b_theta=%.10f",
                 slate->b_rpt[0], slate->b_rpt[1], slate->b_rpt[2]);
        LOG_INFO("  Expected: b_r=%.10f, b_phi=%.10f, b_theta=%.10f",
                 expected_B[0], expected_B[1], expected_B[2]);
        LOG_INFO("  Dot product: %.10f (threshold: %.10f)", dot_product,
                 DOT_PRODUCT_THRESHOLD);

        if (test_passed)
            passed++;
    }

    LOG_INFO("B field reference tests: %d/%d passed", passed, num_tests);
}

void test_b_field_mapping(slate_t *slate)
{
    LOG_INFO("Generating B field mapping data...");

    const float ALTITUDE = 400.0f; // km
    const float LAT_STEP = 5.0f;   // degrees
    const float LON_STEP = 10.0f;  // degrees

    printf("Altitude,Latitude,Longitude,B_r,B_phi,B_theta,B_magnitude\n");

    for (float lat = -89.9f; lat <= 89.9f; lat += LAT_STEP)
    {
        for (float lon = -179.9f; lon <= 179.9f; lon += LON_STEP)
        {
            slate->gps_lat = lat;
            slate->gps_lon = lon;

            compute_B(slate);

            float magnitude = sqrtf(slate->b_rpt[0] * slate->b_rpt[0] +
                                    slate->b_rpt[1] * slate->b_rpt[1] +
                                    slate->b_rpt[2] * slate->b_rpt[2]);

            printf("%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n", ALTITUDE, lat,
                   lon, slate->b_rpt[0], slate->b_rpt[1], slate->b_rpt[2],
                   magnitude);
        }
    }

    LOG_INFO("B field mapping complete");
}

void test_b_field_ecef_conversion(slate_t *slate)
{
    LOG_INFO("Testing B field ECEF conversion...");

    // Test point: North Pole at satellite mean altitude
    slate->gps_lat = 89.9f;
    slate->gps_lon = 0.0f;
    slate->gps_alt = 400.0f;

    compute_B(slate);

    const float MAGNITUDE_TOLERANCE =
        1e-6f; // Normalized vectors should have magnitude 1
    const float DOT_PRODUCT_THRESHOLD = 0.99f; // Directional accuracy threshold

    // Check that both RTP and ECEF fields are populated and normalized
    float rtp_magnitude = sqrtf(slate->b_rpt[0] * slate->b_rpt[0] +
                                slate->b_rpt[1] * slate->b_rpt[1] +
                                slate->b_rpt[2] * slate->b_rpt[2]);

    float ecef_magnitude = sqrtf(slate->b_ecef[0] * slate->b_ecef[0] +
                                 slate->b_ecef[1] * slate->b_ecef[1] +
                                 slate->b_ecef[2] * slate->b_ecef[2]);

    float enu_magnitude = sqrtf(slate->b_enu[0] * slate->b_enu[0] +
                                slate->b_enu[1] * slate->b_enu[1] +
                                slate->b_enu[2] * slate->b_enu[2]);

    // Check normalization (magnitudes should be 1.0)
    bool rtp_normalized = fabsf(rtp_magnitude - 1.0f) < MAGNITUDE_TOLERANCE;
    bool ecef_normalized = fabsf(ecef_magnitude - 1.0f) < MAGNITUDE_TOLERANCE;
    bool enu_normalized = fabsf(enu_magnitude - 1.0f) < MAGNITUDE_TOLERANCE;

    // Test frame transforms preserve direction (dot product should be close to
    // 1)
    float rtp_enu_dot = slate->b_rpt[0] * slate->b_enu[2] + // b_r -> Up
                        slate->b_rpt[1] * slate->b_enu[0] + // b_phi -> East
                        slate->b_rpt[2] * slate->b_enu[1];  // b_theta -> North

    bool direction_preserved = rtp_enu_dot >= DOT_PRODUCT_THRESHOLD;

    bool all_tests_passed = rtp_normalized && ecef_normalized &&
                            enu_normalized && direction_preserved;

    LOG_INFO("RTP magnitude: %.10f (normalized: %s)", rtp_magnitude,
             rtp_normalized ? "PASS" : "FAIL");
    LOG_INFO("ECEF magnitude: %.10f (normalized: %s)", ecef_magnitude,
             ecef_normalized ? "PASS" : "FAIL");
    LOG_INFO("ENU magnitude: %.10f (normalized: %s)", enu_magnitude,
             enu_normalized ? "PASS" : "FAIL");
    LOG_INFO("RTP-ENU direction dot product: %.10f (threshold: %.10f)",
             rtp_enu_dot, DOT_PRODUCT_THRESHOLD);
    LOG_INFO("ECEF conversion test: %s", all_tests_passed ? "PASS" : "FAIL");

    LOG_INFO("b_rpt:  [%.10f, %.10f, %.10f]", slate->b_rpt[0], slate->b_rpt[1],
             slate->b_rpt[2]);
    LOG_INFO("b_enu:  [%.10f, %.10f, %.10f]", slate->b_enu[0], slate->b_enu[1],
             slate->b_enu[2]);
    LOG_INFO("b_ecef: [%.10f, %.10f, %.10f]", slate->b_ecef[0],
             slate->b_ecef[1], slate->b_ecef[2]);
}
#endif