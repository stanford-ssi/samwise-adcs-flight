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
#define MAX_ORDER 5

// Forward declare legendre polynomial functions
float legendre(int n, int m, float x);
float d_legendre(int n, int m, float x);

float g[MAX_ORDER + 1][MAX_ORDER + 1] = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},              // n=0
    {-29350.0, -1410.3, 0.0, 0.0, 0.0, 0.0},     // n=1
    {-2556.2, 2950.9, 1648.7, 0.0, 0.0, 0.0},    // n=2
    {1360.9, -2404.2, 1243.8, 453.4, 0.0, 0.0},  // n=3
    {894.7, 799.6, 55.8, -281.1, 12.0, 0.0},     // n=4
    {-232.9, 369.0, 187.2, -138.7, -141.9, 20.9} // n=5
};

float h[MAX_ORDER + 1][MAX_ORDER + 1] = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},           // n=0
    {0.0, 4545.5, 0.0, 0.0, 0.0, 0.0},        // n=1
    {0.0, -3133.6, -814.2, 0.0, 0.0, 0.0},    // n=2
    {0.0, -56.9, 237.6, -549.6, 0.0, 0.0},    // n=3
    {0.0, 278.6, -134.0, 212.0, -375.4, 0.0}, // n=4
    {0.0, 45.3, 220.0, -122.9, 42.9, 106.2}   // n=5
};

void compute_B(slate_t *slate)
{
    float alt = slate->geodetic[0]; // altitude (km)
    float lat = slate->geodetic[1]; // longitude: -180 to 180
    float lon = slate->geodetic[2]; // latitude: -90 to 90 exclusive

    // Convert to spherical coordinates (theta is colatitude)
    float phi = (90.0f - lat) * M_PI / 180.0f; // colatitude in radians
    float theta = lon * M_PI / 180.0f;         // longitude in radians

    float Br = 0;
    float Btheta = 0;
    float Bphi = 0;

    float r_ratio = R_E / (R_E + alt);

    // Cache sin/cos phi terms
    float sin_mphi[MAX_ORDER + 1];
    float cos_mphi[MAX_ORDER + 1];
    for (int m = 0; m <= MAX_ORDER; m++)
    {
        sin_mphi[m] = sin(m * phi); // try precomputing instead later
        cos_mphi[m] = cos(m * phi);
    }

    // Compute field components
    for (int n = 1; n <= MAX_ORDER; n++)
    {
        float r_ratio_n = pow(r_ratio, n + 2);
        for (int m = 0; m <= n; m++)
        {
            float P = legendre(n, m, cos(theta));
            float dP = d_legendre(n, m, cos(theta));

            float term =
                r_ratio_n * (g[n][m] * cos_mphi[m] + h[n][m] * sin_mphi[m]);

            Br += (n + 1) * P * term;
            Btheta += -dP * term;
            Bphi += m * P / sin(theta) *
                    (g[n][m] * sin_mphi[m] - h[n][m] * cos_mphi[m]);

            // Handle poles separately to avoid division by zero
            if (fabs(sin(phi)) > 1e-10)
            {
                Bphi += m * P / sin(phi) *
                        (g[n][m] * sin_mphi[m] - h[n][m] * cos_mphi[m]);
            }
        }
    }

    // Create and return float3 vector
    slate->B_est = float3(Br, Btheta, Bphi);
}

float legendre(int n, int m, float x)
{
    float pmm = 1.0;

    if (m > 0)
    {
        float somx2 = sqrt((1.0 - x) * (1.0 + x));
        float fact = 1.0;
        for (int i = 1; i <= m; i++)
        {
            pmm *= -somx2 * sqrt((2 * i + 1) /
                                 (2.0 * i)); // Schmidt normalization factor
            fact += 2.0;
        }
    }

    if (n == m)
    {
        return pmm;
    }

    float pmmp1 = x * sqrt(2 * m + 3) * pmm; // Include normalization
    if (n == m + 1)
    {
        return pmmp1;
    }

    float pnm = 0.0;
    for (int k = m + 2; k <= n; k++)
    {
        float dk = k;
        float dm = m;
        float scalar = sqrt((4.0 * k * k - 1.0) / (k * k - m * m));
        pnm = (x * pmmp1 * scalar -
               pmm * sqrt(((2.0 * k + 1.0) * ((k - 1.0) * (k - 1.0) - m * m)) /
                          ((2.0 * k - 3.0) * (k * k - m * m)))) /
              sqrt((dk + dm) * (dk - dm));
        pmm = pmmp1;
        pmmp1 = pnm;
    }

    return pnm;
}

float d_legendre(int n, int m, float x)
{
    // Special case for x = ±1
    if (fabs(x) == 1.0)
    {
        return 0.0;
    }

    // Compute derivative using relationship with polynomials of adjacent degree
    float factor = sqrt(1.0 - x * x);

    if (n == m)
    {
        return n * x * legendre(n, m, x) / factor;
    }

    return (n * x * legendre(n, m, x) - (n + m) * legendre(n - 1, m, x)) /
           factor;
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