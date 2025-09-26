/**
 * @author Lundeen Cahilly
 * @date 2025-08-17
 *
 * Magnetic field model tests
 */

#include "b_field.h"
#include "constants.h"
#include "gnc/models/b_field.h"
#include "macros.h"
#include <cmath>

void test_b_field_reference_points(slate_t *slate)
{
    LOG_INFO("Testing B field reference points...");

    const float DOT_PRODUCT_THRESHOLD = 0.99f; // Directional accuracy threshold

    struct TestPoint
    {
        float lat, lon, alt;
        float expected_Br, expected_Bphi, expected_Btheta, expected_mag;
        const char *name;
    };

    // Reference values from IGRF calculator at mean satellite altitude
    TestPoint test_points[] = {
        {89.9f, 0.0f, SATELLITE_MEAN_ALTITUDE, -56835.0f, 439.0f, -1783.0f,
         56864.0f, "North Pole"},
        {-89.9f, 0.0f, SATELLITE_MEAN_ALTITUDE, 51612.0f, -8771.0f, -14391.0f,
         54294.0f, "South Pole"},
        {0.0f, 0.0f, SATELLITE_MEAN_ALTITUDE, 15997.0f, -1926.0f, -27457.0f,
         31835.0f, "Equator at Greenwich"},
        {45.0f, -75.0f, SATELLITE_MEAN_ALTITUDE, -42838.0f, -3417.0f, -15976.0f,
         45848.0f, "Mid-latitude point"},
        {-30.0f, -45.0f, SATELLITE_MEAN_ALTITUDE, 16572.0f, -5499.0f, -14603.0f,
         22762.0f, "South Atlantic Anomaly"}};

    int num_tests = sizeof(test_points) / sizeof(TestPoint);
    int passed = 0;

    for (int i = 0; i < num_tests; i++)
    {
        slate->gps_lat = test_points[i].lat;
        slate->gps_lon = test_points[i].lon;

        compute_B(slate);

        // Normalize expected vector for direction comparison
        float3 expected_B = {test_points[i].expected_Br,
                             test_points[i].expected_Bphi,
                             test_points[i].expected_Btheta};
        float expected_magnitude = sqrtf(expected_B[0] * expected_B[0] +
                                         expected_B[1] * expected_B[1] +
                                         expected_B[2] * expected_B[2]);
        expected_B = expected_B / expected_magnitude;

        // Compute dot product for directional comparison
        float dot_product = slate->B_est_rpt[0] * expected_B[0] +
                            slate->B_est_rpt[1] * expected_B[1] +
                            slate->B_est_rpt[2] * expected_B[2];

        bool test_passed = dot_product >= DOT_PRODUCT_THRESHOLD;

        LOG_INFO("%s: %s", test_points[i].name, test_passed ? "PASS" : "FAIL");
        LOG_INFO("  Computed: Br=%.10f, Bphi=%.10f, Btheta=%.10f",
                 slate->B_est_rpt[0], slate->B_est_rpt[1], slate->B_est_rpt[2]);
        LOG_INFO("  Expected: Br=%.10f, Bphi=%.10f, Btheta=%.10f",
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

    const float ALTITUDE = SATELLITE_MEAN_ALTITUDE; // km
    const float LAT_STEP = 5.0f;                    // degrees
    const float LON_STEP = 10.0f;                   // degrees

    printf("Altitude,Latitude,Longitude,B_r,B_phi,B_theta,B_magnitude\n");

    for (float lat = -89.9f; lat <= 89.9f; lat += LAT_STEP)
    {
        for (float lon = -179.9f; lon <= 179.9f; lon += LON_STEP)
        {
            slate->gps_lat = lat;
            slate->gps_lon = lon;

            compute_B(slate);

            float magnitude = sqrtf(slate->B_est_rpt[0] * slate->B_est_rpt[0] +
                                    slate->B_est_rpt[1] * slate->B_est_rpt[1] +
                                    slate->B_est_rpt[2] * slate->B_est_rpt[2]);

            printf("%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n", ALTITUDE, lat,
                   lon, slate->B_est_rpt[0], slate->B_est_rpt[1],
                   slate->B_est_rpt[2], magnitude);
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

    compute_B(slate);

    const float MAGNITUDE_TOLERANCE =
        1e-6f; // Normalized vectors should have magnitude 1
    const float DOT_PRODUCT_THRESHOLD = 0.99f; // Directional accuracy threshold

    // Check that both RTP and ECEF fields are populated and normalized
    float rtp_magnitude = sqrtf(slate->B_est_rpt[0] * slate->B_est_rpt[0] +
                                slate->B_est_rpt[1] * slate->B_est_rpt[1] +
                                slate->B_est_rpt[2] * slate->B_est_rpt[2]);

    float ecef_magnitude = sqrtf(slate->B_est_ecef[0] * slate->B_est_ecef[0] +
                                 slate->B_est_ecef[1] * slate->B_est_ecef[1] +
                                 slate->B_est_ecef[2] * slate->B_est_ecef[2]);

    float enu_magnitude = sqrtf(slate->B_est_enu[0] * slate->B_est_enu[0] +
                                slate->B_est_enu[1] * slate->B_est_enu[1] +
                                slate->B_est_enu[2] * slate->B_est_enu[2]);

    // Check normalization (magnitudes should be 1.0)
    bool rtp_normalized = fabsf(rtp_magnitude - 1.0f) < MAGNITUDE_TOLERANCE;
    bool ecef_normalized = fabsf(ecef_magnitude - 1.0f) < MAGNITUDE_TOLERANCE;
    bool enu_normalized = fabsf(enu_magnitude - 1.0f) < MAGNITUDE_TOLERANCE;

    // Test frame transforms preserve direction (dot product should be close to
    // 1)
    float rtp_enu_dot =
        slate->B_est_rpt[0] * slate->B_est_enu[2] + // Br -> Up
        slate->B_est_rpt[1] * slate->B_est_enu[0] + // Bphi -> East
        slate->B_est_rpt[2] * slate->B_est_enu[1];  // Btheta -> North

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

    LOG_INFO("B_est_rpt:  [%.10f, %.10f, %.10f]", slate->B_est_rpt[0],
             slate->B_est_rpt[1], slate->B_est_rpt[2]);
    LOG_INFO("B_est_enu:  [%.10f, %.10f, %.10f]", slate->B_est_enu[0],
             slate->B_est_enu[1], slate->B_est_enu[2]);
    LOG_INFO("B_est_ecef: [%.10f, %.10f, %.10f]", slate->B_est_ecef[0],
             slate->B_est_ecef[1], slate->B_est_ecef[2]);
}