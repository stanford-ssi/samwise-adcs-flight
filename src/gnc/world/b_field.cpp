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
    {1.000000f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},                     // n=0
    {1.000000f, 1.000000f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},                // n=1
    {1.000000f, 0.577350f, 0.288675f, 0.0f, 0.0f, 0.0f, 0.0f},           // n=2
    {1.000000f, 0.408248f, 0.129099f, 0.052705f, 0.0f, 0.0f, 0.0f},      // n=3
    {1.000000f, 0.316228f, 0.074536f, 0.019920f, 0.007043f, 0.0f, 0.0f}, // n=4
    {1.000000f, 0.258199f, 0.048795f, 0.009960f, 0.002348f, 0.000742f,
     0.0f}, // n=5
    {1.000000f, 0.218218f, 0.034993f, 0.005968f, 0.001104f, 0.000246f,
     0.000071f} // n=6
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
    float pmm = SCHMIDT_FACTORS[n][m];

    if (m > 0)
    {
        float somx2 = sqrt((1.0f - x) * (1.0f + x));
        for (int i = 1; i <= m; i++)
        {
            pmm *= -somx2;
        }
    }

    if (n == m)
        return pmm;

    float pmmp1 = x * (2.0f * m + 1.0f) * pmm;
    if (n == m + 1)
        return pmmp1;

    float pnm;
    for (int k = m + 2; k <= n; k++)
    {
        pnm = ((2.0f * k - 1.0f) * x * pmmp1 - (k + m - 1.0f) * pmm) / (k - m);
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