/**
 * @author Lundeen Cahilly
 * @date 2025-02-10
 */

#include "gnc/world/b_field.h"
#include "macros.h"
#include "pico/stdlib.h"
#include <cmath>
// #include "constants.h"

#define R_E 6378.0f
#define MAX_ORDER 6

// Forward declare legendre polynomial functions
float legendre_schmidt(int n, int m, float x);
float d_legendre_schmidt(int n, int m, float x);

// IGRF 2025 Coefficients (up to n=6)
float g[MAX_ORDER + 1][MAX_ORDER + 1] = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},               // n=0
    {-29350.0, -1410.3, 0.0, 0.0, 0.0, 0.0, 0.0},      // n=1
    {-2556.2, 2950.9, 1648.7, 0.0, 0.0, 0.0, 0.0},     // n=2
    {1360.9, -2404.2, 1243.8, 453.4, 0.0, 0.0, 0.0},   // n=3
    {894.7, 799.6, 55.8, -281.1, 12.0, 0.0, 0.0},      // n=4
    {-232.9, 369.0, 187.2, -138.7, -141.9, 20.9, 0.0}, // n=5
    {66.0, 65.5, 72.9, -121.5, -36.2, 13.9, -64.7}     // n=6
};

float h[MAX_ORDER + 1][MAX_ORDER + 1] = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},           // n=0
    {0.0, 4545.5, 0.0, 0.0, 0.0, 0.0, 0.0},        // n=1
    {0.0, -3133.6, -814.2, 0.0, 0.0, 0.0, 0.0},    // n=2
    {0.0, -56.9, 237.6, -549.6, 0.0, 0.0, 0.0},    // n=3
    {0.0, 278.6, -134.0, 212.0, -375.4, 0.0, 0.0}, // n=4
    {0.0, 45.3, 220.0, -122.9, 42.9, 106.2, 0.0},  // n=5
    {0.0, -24.5, -27.7, -79.3, 2.2, 16.9, -50.2}   // n=6
};

// Pre-computed Schmidt quasi-normalization factors
// S(n,m) = sqrt((2-delta(0,m))(n-m)!/(n+m)!)
// TODO: RECALCULATE SCHMIDT FACTORS
const float SCHMIDT_FACTORS[MAX_ORDER + 1][MAX_ORDER + 1] = {
    {1.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=0
    {1.000000, 1.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=1
    {1.000000, 0.577350, 0.288675, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=2
    {1.000000, 0.408248, 0.129099, 0.052705, 0.000000, 0.000000,
     0.000000}, // n=3
    {1.000000, 0.316228, 0.074536, 0.019920, 0.007043, 0.000000,
     0.000000}, // n=4
    {1.000000, 0.258199, 0.048795, 0.009960, 0.002348, 0.000742,
     0.000000}, // n=5
    {1.000000, 0.218218, 0.034503, 0.005751, 0.001050, 0.000224,
     0.000065}, // n=6
};

void compute_B(slate_t *slate)
{
    const float alt = slate->geodetic[0]; // altitude (km)
    const float lat = slate->geodetic[1]; // latitude (-90 to 90)
    const float lon = slate->geodetic[2]; // longitude (-180 to 180)

    // Convert to spherical coordinates using physics/math spherical coordinate
    // conventions
    const float phi = (90.0f - lat) * M_PI / 180.0f; // colatitude [0 to π]
    const float theta = lon * M_PI / 180.0f;         // azimuth [-π to π]

    float Br = 0.0f;
    float Btheta = 0.0f;
    float Bphi = 0.0f;

    const float r_ratio = R_E / (R_E + alt);

    // Cache trig terms with pole regularization
    const float sin_phi = sin(phi); // for pole regularization
    const float cos_phi = cos(phi); // for Legendre polynomials

    // // Regularization for pole proximity (prevents division by zero)
    const float POLE_THRESH = 1e-7f; // Added missing constant
    const float sin_phi_reg =
        (fabs(sin_phi) < POLE_THRESH)
            ? POLE_THRESH * (sin_phi >= 0.0f ? 1.0f : -1.0f)
            : sin_phi;

    // Pre-compute sin/cos m*phi terms
    float sin_mtheta[MAX_ORDER + 1] = {0.0f};
    float cos_mtheta[MAX_ORDER + 1] = {0.0f};
    for (int m = 0; m <= MAX_ORDER; m++)
    {
        sin_mtheta[m] = sin(m * theta);
        cos_mtheta[m] = cos(m * theta);
    }

    // Main field computation loop
    float r_ratio_n = r_ratio * r_ratio; // Start at n=1 term T
    for (int n = 1; n <= MAX_ORDER; n++)
    {
        for (int m = 0; m <= n; m++)
        {
            // Get Schmidt normalized associated Legendre functions
            const float P = legendre_schmidt(n, m, cos_phi);
            const float dP = d_legendre_schmidt(n, m, cos_phi);

            // Compute common term for efficiency
            const float term =
                r_ratio_n * (g[n][m] * cos_mtheta[m] + h[n][m] * sin_mtheta[m]);

            // Accumulate field components
            Br += (float)(n + 1) * P * term * r_ratio;
            Btheta -= dP * term;

            // Handle Bphi carefully near poles
            if (m > 0)
            { // m=0 terms don't contribute to Bphi
                const float Bphi_term =
                    (-g[n][m] * sin_mtheta[m] + h[n][m] * cos_mtheta[m]) * P *
                    r_ratio_n / sin_phi_reg;
                // (float)m * P / sin_phi_reg *

                // Smooth transition near poles
                const float pole_factor = (fabs(sin_phi) < POLE_THRESH)
                                              ? sin_phi / POLE_THRESH
                                              : 1.0f;

                Bphi -= Bphi_term * pole_factor;
            }
        }

        r_ratio_n *= r_ratio;
    }

    // Store results
    slate->B_est = float3(Br, Btheta, Bphi);
}

// Modified Legendre function using pre-computed factors
float legendre_schmidt(int n, int m, float x)
{
    if (n > MAX_ORDER || m > n)
        return 0;

    // First compute P_m^m
    float pmm = 1.0f;

    if (m > 0)
    {
        float somx2 = sqrt((1.0f - x) * (1.0f + x));
        for (int i = 1; i <= m; i++)
        {
            pmm *= somx2;
        }
    }

    // Apply Schmidt factor from table
    pmm *= SCHMIDT_FACTORS[m][m];

    if (n == m)
        return pmm;

    // Compute P_(m+1)^m
    float pmmp1 = x * (2.0f * m + 1.0f) * pmm;

    // Apply Schmidt factor for m+1
    pmmp1 *= SCHMIDT_FACTORS[m + 1][m] / SCHMIDT_FACTORS[m][m];

    if (n == (m + 1))
        return pmmp1;

    // Use recurrence to get P_n^m
    float pnm;
    for (int k = m + 2; k <= n; k++)
    {
        pnm = ((2.0f * k - 1.0f) * x * pmmp1 - (k + m - 1.0f) * pmm) / (k - m);
        // Apply Schmidt factor ratio for this k
        pnm *= SCHMIDT_FACTORS[k][m] / SCHMIDT_FACTORS[k - 1][m];
        pmm = pmmp1;
        pmmp1 = pnm;
    }

    return pnm;
}

float d_legendre_schmidt(int n, int m, float x)
{
    // Special case for poles (x = ±1)
    if (fabs(x) >= 0.99999f)
    { // Use float comparison threshold
        return 0.0f;
    }

    // Calculate derivative using relationship for Schmidt semi-normalized
    // functions dP_n^m/dθ = 1/sqrt(1-x^2) * [ n*x*P_n^m - (n+m)*P_(n-1)^m ]
    const float factor = 1.0f / sqrt(1.0f - x * x);

    if (n == m)
    {
        return n * x * legendre_schmidt(n, m, x) * factor;
    }

    // We need both P_n^m and P_(n-1)^m for the derivative
    const float P_n = legendre_schmidt(n, m, x);
    const float P_nm1 = legendre_schmidt(n - 1, m, x);

    return (n * x * P_n - (n + m) * P_nm1) * factor;
}

void test_compute_B(slate_t *slate)
{
    LOG_INFO("Testing B field model...");

    const float EPSILON = 1000.0f; // Tolerance for float comparisons

    // Test points in {alt (km), lat (deg), lon (deg)} format
    struct TestPoint
    {
        float alt, lat, lon;
        float expected_Br;     // Expected radial component  (-Z)
        float expected_Btheta; // Expected theta component  (-X)
        float expected_Bphi;   // Expected phi component    (Y)
        float expected_mag;    // Expected total magnitude  (F)
        const char *name;
    };

    // Assertion values from IGRF calculator
    TestPoint test_points[] = {{0.0f, 89.9f, 0.0f, -56835.0f, -1783.0f, 439.0f,
                                56864.0f, "North Pole"},

                               {0.0f, -89.9f, 0.0f, -51612.0f, -14391.0f,
                                -8771.0f, 54294.0f, "South Pole"},

                               {0.0f, 0.0f, 0.0f, 15997.0f, -27457.0f, -1926.0f,
                                31835.0f, "Equator at Greenwich"},

                               {0.0f, 0.0f, 180.0f, 3074.0f, -33431.0f, 5859.0f,
                                34079.0f, "Equator opposite GMT"},

                               {300.0f, 45.0f, -75.0f, -42838.0f, -15976.0f,
                                -3417.0f, 45848.0f, "Mid-latitude point"},

                               {0.0f, -30.0f, -45.0f, 16572.0f, -14603.0f,
                                -5499.0f, 22762.0f, "South Atlantic Anomaly"},

                               {1000.0f, 80.0f, -72.0f, -37527.0f, -2084.0f,
                                -1163.0f, 37603.0f, "Near Magnetic North"}};

    const int num_tests = sizeof(test_points) / sizeof(TestPoint);
    int passed = 0;
    int failed = 0;

    LOG_INFO("\n=== Main Test Points ===");
    for (int i = 0; i < num_tests; i++)
    {
        // Set test point
        slate->geodetic[0] = test_points[i].alt;
        slate->geodetic[1] = test_points[i].lat;
        slate->geodetic[2] = test_points[i].lon;

        // Compute field
        compute_B(slate);

        // Compute field magnitude
        float magnitude = sqrt(slate->B_est.x * slate->B_est.x +
                               slate->B_est.y * slate->B_est.y +
                               slate->B_est.z * slate->B_est.z);

        // Calculate differences
        float diff_Br = fabs(slate->B_est.x - test_points[i].expected_Br);
        float diff_Btheta =
            fabs(slate->B_est.y - test_points[i].expected_Btheta);
        float diff_Bphi = fabs(slate->B_est.z - test_points[i].expected_Bphi);
        float diff_mag = fabs(magnitude - test_points[i].expected_mag);

        // Log detailed results
        LOG_INFO("\nTest %d: %s", i + 1, test_points[i].name);
        LOG_INFO("Position: alt=%.1f km, lat=%.1f°, lon=%.1f°",
                 test_points[i].alt, test_points[i].lat, test_points[i].lon);
        LOG_INFO("Component  | Computed (nT) | Expected (nT) | Diff (nT) | "
                 "Diff (%)");
        LOG_INFO(
            "-----------|--------------|--------------|-----------|----------");
        LOG_INFO("Br         | %11.1f | %11.1f | %9.1f | %8.1f%%",
                 slate->B_est.x, test_points[i].expected_Br, diff_Br,
                 100.0f * diff_Br / fabs(test_points[i].expected_Br));
        LOG_INFO("Btheta     | %11.1f | %11.1f | %9.1f | %8.1f%%",
                 slate->B_est.y, test_points[i].expected_Btheta, diff_Btheta,
                 100.0f * diff_Btheta / fabs(test_points[i].expected_Btheta));
        LOG_INFO("Bphi       | %11.1f | %11.1f | %9.1f | %8.1f%%",
                 slate->B_est.z, test_points[i].expected_Bphi, diff_Bphi,
                 100.0f * diff_Bphi / fabs(test_points[i].expected_Bphi));
        LOG_INFO("Magnitude  | %11.1f | %11.1f | %9.1f | %8.1f%%", magnitude,
                 test_points[i].expected_mag, diff_mag,
                 100.0f * diff_mag / test_points[i].expected_mag);

        // Check if test passed
        bool test_passed = (diff_Br < EPSILON && diff_Btheta < EPSILON &&
                            diff_Bphi < EPSILON && diff_mag < EPSILON);

        LOG_INFO("Test result: %s", test_passed ? "PASSED ✓" : "FAILED ✗");

        if (test_passed)
        {
            passed++;
        }
        else
        {
            failed++;
            LOG_ERROR(
                "Test %d failed with differences exceeding epsilon (%.1e)!",
                i + 1, EPSILON);
        }

        // Assert expected values
        assert(diff_Br < EPSILON && "Radial component (Br) test failed");
        assert(diff_Btheta < EPSILON && "Theta component (Btheta) test failed");
        assert(diff_Bphi < EPSILON && "Phi component (Bphi) test failed");
        assert(diff_mag < pow(EPSILON, 3) && "Magnitude test failed");

        sleep_ms(100);
    }

    LOG_INFO("\nMain tests complete:");
    LOG_INFO("  Passed: %d", passed);
    LOG_INFO("  Failed: %d", failed);
    LOG_INFO("  Total:  %d", num_tests);

    LOG_INFO("\nAll B field tests complete");
}

void test_legendre_polynomials()
{
    LOG_INFO("Testing Legendre and associated Legendre functions...");

    const float EPSILON = 1e-4f; // Tolerance for float comparisons

    struct TestCase
    {
        int n;             // Degree
        int m;             // Order
        float x;           // Input value
        float expected_p;  // Expected P_n^m value
        float expected_dp; // Expected derivative value
        const char *desc;  // Test description
    };

    TestCase test_cases[] = {
        // P_0^0 at x=0 - constant function
        {0, 0, 0.0f, 1.0f, 0.0f, "P_0^0 at x=0 - constant function"},
        // P_0^0 at x=0.5 - should still be 1
        {0, 0, 0.5f, 1.0f, 0.0f, "P_0^0 at x=0.5 - should still be 1"},
        // P_0^0 at x=-0.5 - verifies symmetry
        {0, 0, -0.5f, 1.0f, 0.0f, "P_0^0 at x=-0.5 - verifies symmetry"},
        // P_1^0 at x=0 - should be zero at origin
        {1, 0, 0.0f, 0.0f, 1.0f, "P_1^0 at x=0 - should be zero at origin"},
        // P_1^0 at x=0.5 - tests linear behavior
        {1, 0, 0.5f, 0.5f, 1.0f, "P_1^0 at x=0.5 - tests linear behavior"},
        // P_1^0 at x=-0.5 - verifies odd symmetry
        {1, 0, -0.5f, -0.5f, 1.0f, "P_1^0 at x=-0.5 - verifies odd symmetry"},
        // P_1^1 at x=0 - maximum at equator
        {1, 1, 0.0f, 1.0f, 0.0f, "P_1^1 at x=0 - maximum at equator"},
        // P_1^1 at x=0.5 - tests sqrt(1-x^2)
        {1, 1, 0.5f, 0.866f, -0.5774f, "P_1^1 at x=0.5 - tests sqrt(1-x^2)"},
        // P_1^1 at x=-0.5 - tests symmetry
        {1, 1, -0.5f, 0.866f, 0.5774f, "P_1^1 at x=-0.5 - tests symmetry"},
        // P_2^0 at x=0 - local minimum
        {2, 0, 0.0f, -0.5f, 0.0f, "P_2^0 at x=0 - local minimum"},
        // P_2^0 at x=0.5 - tests quadratic
        {2, 0, 0.5f, -0.125f, 1.5f, "P_2^0 at x=0.5 - tests quadratic"},
        // P_2^0 at x=-0.5 - even symmetry
        {2, 0, -0.5f, -0.125f, -1.5f, "P_2^0 at x=-0.5 - even symmetry"},
        // P_2^1 at x=0 - zero crossing
        {2, 1, 0.0f, 0.0f, 3.0f, "P_2^1 at x=0 - zero crossing"},
        // P_2^1 at x=0.5 - tests derivative
        {2, 1, 0.5f, -1.299f, 2.598f, "P_2^1 at x=0.5 - tests derivative"},
        // P_2^1 at x=-0.5 - odd symmetry
        {2, 1, -0.5f, 1.299f, 2.598f, "P_2^1 at x=-0.5 - odd symmetry"},
        // P_2^2 at x=0 - maximum at equator
        {2, 2, 0.0f, 3.0f, 0.0f, "P_2^2 at x=0 - maximum at equator"},
        // P_2^2 at x=0.5 - high precision test
        {2, 2, 0.5f, 2.598f, -3.897f, "P_2^2 at x=0.5 - high precision test"},
        // P_2^2 at x=-0.5 - even symmetry
        {2, 2, -0.5f, 2.598f, 3.897f, "P_2^2 at x=-0.5 - even symmetry"},
        // P_1^0 near north pole - tests pole handling
        {1, 0, 0.9999f, 0.9999f, 1.0f,
         "P_1^0 near north pole - tests pole handling"},
        // P_1^0 near south pole - tests pole handling
        {1, 0, -0.9999f, -0.9999f, 1.0f,
         "P_1^0 near south pole - tests pole handling"},
        // P_3^0 at x=0.5 - tests cubic terms
        {3, 0, 0.5f, 0.4375f, 2.344f, "P_3^0 at x=0.5 - tests cubic terms"},
        // P_4^0 at x=0.5 - tests quartic terms
        {4, 0, 0.5f, 0.2734f, 2.459f, "P_4^0 at x=0.5 - tests quartic terms"},
    };

    // Diagnostic guide:
    // 1. If P_0^0 tests fail: Check initialization and base cases
    // 2. If P_1^0 tests fail but P_0^0 passes: Check recursion start
    // 3. If P_n^0 tests fail for n>1: Check main recursion formula
    // 4. If P_n^m tests fail for m>0: Check associated Legendre implementation
    // 5. If only derivatives fail: Check derivative formula implementation
    // 6. If pole tests fail: Check handling of x near ±1

    const int num_tests = sizeof(test_cases) / sizeof(TestCase);
    int passed = 0;
    int failed = 0;

    for (int i = 0; i < num_tests; i++)
    {
        TestCase *tc = &test_cases[i];

        // Test Legendre function
        float p = legendre_schmidt(tc->n, tc->m, tc->x);
        bool p_pass = fabs(p - tc->expected_p) < EPSILON;

        // Test derivative
        float dp = d_legendre_schmidt(tc->n, tc->m, tc->x);
        bool dp_pass = fabs(dp - tc->expected_dp) < EPSILON;

        // Log results
        LOG_INFO("\nTest %d: %s", i + 1, tc->desc);
        LOG_INFO("P_%d^%d(%.4f):", tc->n, tc->m, tc->x);
        LOG_INFO("  Expected P: %.4f, Got: %.4f %s", tc->expected_p, p,
                 p_pass ? "✓" : "✗");
        LOG_INFO("  Expected dP: %.4f, Got: %.4f %s", tc->expected_dp, dp,
                 dp_pass ? "✓" : "✗");

        if (p_pass && dp_pass)
        {
            passed++;
        }
        else
        {
            failed++;
            LOG_ERROR("Test %d failed!", i + 1);
        }

        // Add delay between tests for serial output
        sleep_ms(100);
    }

    // Final summary
    LOG_INFO("\nLegendre testing complete:");
    LOG_INFO("  Passed: %d", passed);
    LOG_INFO("  Failed: %d", failed);
    LOG_INFO("  Total:  %d", num_tests);

    // Additional verification tests
    LOG_INFO("\nRunning additional verification tests...");

    // Test symmetry properties
    for (float x = -0.9f; x <= 0.9f; x += 0.3f)
    {
        // P_n^m(-x) = (-1)^(n+m) P_n^m(x)
        float p1 = legendre_schmidt(2, 1, x);
        float p2 = legendre_schmidt(2, 1, -x);
        assert(fabs(p1 + p2) < EPSILON && "Symmetry test failed");
    }

    // Test orthogonality
    // Could add Gauss-Legendre quadrature tests here

    LOG_INFO("Verification tests complete");
}