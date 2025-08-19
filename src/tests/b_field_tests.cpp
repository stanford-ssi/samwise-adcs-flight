/**
 * @author Lundeen Cahilly
 * @date 2025-08-17
 *
 * Magnetic field model tests
 */

#include "b_field_tests.h"
#include "gnc/world/b_field.h"
#include "macros.h"
#include <cmath>

void test_b_field_reference_points(slate_t *slate)
{
    LOG_INFO("Testing B field reference points...");

    const float EPSILON = 1000.0f; // nT tolerance

    struct TestPoint
    {
        float alt, lat, lon;
        float expected_Br, expected_Bphi, expected_Btheta, expected_mag;
        const char *name;
    };

    // Reference values from IGRF calculator
    TestPoint test_points[] = {{0.0f, 89.9f, 0.0f, -56835.0f, 439.0f, -1783.0f,
                                56864.0f, "North Pole"},
                               {0.0f, -89.9f, 0.0f, 51612.0f, -8771.0f,
                                -14391.0f, 54294.0f, "South Pole"},
                               {0.0f, 0.0f, 0.0f, 15997.0f, -1926.0f, -27457.0f,
                                31835.0f, "Equator at Greenwich"},
                               {300.0f, 45.0f, -75.0f, -42838.0f, -3417.0f,
                                -15976.0f, 45848.0f, "Mid-latitude point"},
                               {0.0f, -30.0f, -45.0f, 16572.0f, -5499.0f,
                                -14603.0f, 22762.0f, "South Atlantic Anomaly"}};

    int num_tests = sizeof(test_points) / sizeof(TestPoint);
    int passed = 0;

    for (int i = 0; i < num_tests; i++)
    {
        slate->geodetic_lat_lon_alt[0] = test_points[i].alt;
        slate->geodetic_lat_lon_alt[1] = test_points[i].lat;
        slate->geodetic_lat_lon_alt[2] = test_points[i].lon;

        compute_B(slate);

        float magnitude = sqrt(slate->B_est[0] * slate->B_est[0] +
                               slate->B_est[1] * slate->B_est[1] +
                               slate->B_est[2] * slate->B_est[2]);

        float diff_Br = fabs(slate->B_est[0] - test_points[i].expected_Br);
        float diff_Bphi = fabs(slate->B_est[1] - test_points[i].expected_Bphi);
        float diff_Btheta =
            fabs(slate->B_est[2] - test_points[i].expected_Btheta);
        float diff_mag = fabs(magnitude - test_points[i].expected_mag);

        bool test_passed = (diff_Br < EPSILON) && (diff_Bphi < EPSILON) &&
                           (diff_Btheta < EPSILON) && (diff_mag < EPSILON);

        LOG_INFO("%s: %s", test_points[i].name, test_passed ? "PASS" : "FAIL");
        LOG_INFO("  Computed: Br=%.1f, Bphi=%.1f, Btheta=%.1f, Mag=%.1f",
                 slate->B_est[0], slate->B_est[1], slate->B_est[2], magnitude);
        LOG_INFO("  Expected: Br=%.1f, Bphi=%.1f, Btheta=%.1f, Mag=%.1f",
                 test_points[i].expected_Br, test_points[i].expected_Bphi,
                 test_points[i].expected_Btheta, test_points[i].expected_mag);
        LOG_INFO("  Diff:     Br=%.1f, Bphi=%.1f, Btheta=%.1f, Mag=%.1f",
                 diff_Br, diff_Bphi, diff_Btheta, diff_mag);

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
            slate->geodetic_lat_lon_alt[0] = ALTITUDE;
            slate->geodetic_lat_lon_alt[1] = lat;
            slate->geodetic_lat_lon_alt[2] = lon;

            compute_B(slate);

            float magnitude = sqrt(slate->B_est[0] * slate->B_est[0] +
                                   slate->B_est[1] * slate->B_est[1] +
                                   slate->B_est[2] * slate->B_est[2]);

            printf("%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f\n", ALTITUDE, lat, lon,
                   slate->B_est[0], slate->B_est[1], slate->B_est[2],
                   magnitude);
        }
    }

    LOG_INFO("B field mapping complete");
}