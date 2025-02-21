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
            Br -= (float)(n + 1) * P * term * r_ratio;
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
            pmm *= -somx2;
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

    // Test points in {alt (km), lat (deg), lon (deg)} format
    struct TestPoint
    {
        float alt, lat, lon;
        const char *name;
    };

    TestPoint test_points[] = {
        {0.0f, 89.9f, 0.0f, "North Pole"},            // North Pole
        {0.0f, -89.9f, 0.0f, "South Pole"},           // South Pole
        {0.0f, 0.0f, 0.0f, "Equator at Greenwich"},   // Equator at 0° longitude
        {0.0f, 0.0f, 180.0f, "Equator opposite GMT"}, // Equator at 180°
        {300.0f, 45.0f, -75.0f, "Mid-latitude point"}, // Northeast US at 300km
        {0.0f, -30.0f, -45.0f, "South Atlantic Anomaly"}, // SAA region
        {1000.0f, 80.0f, -72.0f,
         "Near Magnetic North"}, // Near 2025 magnetic north
    };

    const int num_tests = sizeof(test_points) / sizeof(TestPoint);

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

        // Log results
        LOG_INFO("\nTest %d: %s", i + 1, test_points[i].name);
        LOG_INFO("Position: alt=%.1f km, lat=%.1f°, lon=%.1f°",
                 test_points[i].alt, test_points[i].lat, test_points[i].lon);
        LOG_INFO("B field: Br=%.1f nT, Btheta=%.1f nT, Bphi=%.1f nT",
                 slate->B_est.x, slate->B_est.y, slate->B_est.z);
        LOG_INFO("Total field magnitude: %.1f nT", magnitude);

        // Add delay between tests for serial output
        sleep_ms(100);
    }

    LOG_INFO("\nB field testing complete");
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
        // Test P_0^0 (should be 1.0 everywhere)
        {0, 0, 0.0f, 1.0f, 0.0f, "P_0^0 at x=0"},
        {0, 0, 0.5f, 1.0f, 0.0f, "P_0^0 at x=0.5"},
        {0, 0, -0.5f, 1.0f, 0.0f, "P_0^0 at x=-0.5"},

        // Test P_1^0 (should be x)
        {1, 0, 0.0f, 0.0f, 1.0f, "P_1^0 at x=0"},
        {1, 0, 0.5f, 0.5f, 1.0f, "P_1^0 at x=0.5"},
        {1, 0, -0.5f, -0.5f, 1.0f, "P_1^0 at x=-0.5"},

        // Test P_1^1 (associated Legendre)
        {1, 1, 0.0f, 1.0f, 0.0f, "P_1^1 at x=0"},
        {1, 1, 0.5f, 0.8660f, -0.5774f, "P_1^1 at x=0.5"},
        {1, 1, -0.5f, 0.8660f, 0.5774f, "P_1^1 at x=-0.5"},

        // Test P_2^0
        {2, 0, 0.0f, -0.5f, 0.0f, "P_2^0 at x=0"},
        {2, 0, 0.5f, -0.125f, 1.5f, "P_2^0 at x=0.5"},
        {2, 0, -0.5f, -0.125f, -1.5f, "P_2^0 at x=-0.5"},

        // Test P_2^1
        {2, 1, 0.0f, 0.0f, 3.0f, "P_2^1 at x=0"},
        {2, 1, 0.5f, -1.299f, 2.598f, "P_2^1 at x=0.5"},
        {2, 1, -0.5f, 1.299f, 2.598f, "P_2^1 at x=-0.5"},

        // Test P_2^2
        {2, 2, 0.0f, 3.0f, 0.0f, "P_2^2 at x=0"},
        {2, 2, 0.5f, 2.598f, -3.897f, "P_2^2 at x=0.5"},
        {2, 2, -0.5f, 2.598f, 3.897f, "P_2^2 at x=-0.5"},

        // Test near poles (x ≈ ±1)
        {1, 0, 0.9999f, 0.9999f, 1.0f, "P_1^0 near north pole"},
        {1, 0, -0.9999f, -0.9999f, 1.0f, "P_1^0 near south pole"},

        // Test higher degrees
        {3, 0, 0.5f, 0.4375f, 2.344f, "P_3^0 at x=0.5"},
        {4, 0, 0.5f, 0.2734f, 2.459f, "P_4^0 at x=0.5"}};

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