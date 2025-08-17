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
#define MAX_ORDER 13
#define MODEL_ORDER 11

// Forward declare legendre polynomial functions
float legendre_schmidt(int n, int m, float x);
float d_legendre_schmidt(int n, int m, float x);

// IGRF 2025 Coefficients (up to n=6)
// Indices are n going down and m going left to right
// Units in nanotesla
// There is no n=0 term (that would imply a magnetic monopole)
float g[MAX_ORDER + 1][MAX_ORDER + 1] = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=0
    {-29350, -1410.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=1
    {-2556.2, 2950.9, 1648.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=2
    {1360.9, -2404.2, 1243.8, 453.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0}, // n=3
    {894.7, 799.6, 55.8, -281.1, 12, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=4
    {-232.9, 369, 187.2, -138.7, -141.9, 20.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0}, // n=5
    {64.3, 63.8, 76.7, -115.7, -40.9, 14.9, -60.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=6
    {79.6, -76.9, -8.8, 59.3, 15.8, 2.5, -11.2, 14.3, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=7
    {23.1, 10.9, -17.5, 2, -21.8, 16.9, 14.9, -16.8, 1, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=8
    {4.7, 8, 3, -0.2, -2.5, -13.1, 2.4, 8.6, -8.7, -12.8, 0.0, 0.0, 0.0,
     0.0}, // n=9
    {-1.3, -6.4, 0.2, 2, -1, -0.5, -0.9, 1.5, 0.9, -2.6, -3.9, 0.0, 0.0,
     0.0}, // n=10
    {3, -1.4, -2.5, 2.4, -0.6, 0.0, -0.6, -0.1, 1.1, -1, -0.1, 2.6, 0.0,
     0.0}, // n=11
    {-2, -0.1, 0.4, 1.2, -1.2, 0.6, 0.5, 0.5, -0.1, -0.5, -0.2, -1.2, -0.7,
     0.0}, // n=12
    {0.2, -0.9, 0.6, 0.7, -0.2, 0.5, 0.1, 0.7, 0.0, 0.3, 0.2, 0.4, -0.5,
     -0.4}, // n=13
};

// There is no m=0 term. In m=0 the sin term is zero and
// the cosine term is 1 so only the g term matters.
float h[MAX_ORDER + 1][MAX_ORDER + 1] = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=0
    {0.0, 4545.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=1
    {0.0, -3133.6, -814.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=2
    {0.0, -56.9, 237.6, -549.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=3
    {0.0, 278.6, -134, 212, -375.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=4
    {0.0, 45.3, 220, -122.9, 42.9, 106.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=5
    {0.0, -18.4, 16.8, 48.9, -59.8, 10.9, 72.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=6
    {0.0, -48.9, -14.4, -1, 23.5, -7.4, -25.1, -2.2, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=7
    {0.0, 7.2, -12.6, 11.5, -9.7, 12.7, 0.7, -5.2, 3.9, 0.0, 0.0, 0.0, 0.0,
     0.0}, // n=8
    {0.0, -24.8, 12.1, 8.3, -3.4, -5.3, 7.2, -0.6, 0.8, 9.8, 0.0, 0.0, 0.0,
     0.0}, // n=9
    {0.0, 3.3, 0.1, 2.5, 5.4, -9, 0.4, -4.2, -3.8, 0.9, -9, 0.0, 0.0,
     0.0}, // n=10
    {0.0, 0.0, 2.8, -0.6, 0.1, 0.5, -0.3, -1.2, -1.7, -2.9, -1.8, -2.3, 0.0,
     0.0}, // n=11
    {0.0, -1.2, 0.6, 1, -1.5, 0.0, 0.6, -0.2, 0.8, 0.1, -0.9, 0.1, 0.2,
     0.0}, // n=12
    {0.0, -0.9, 0.7, 1.2, -0.3, -1.3, -0.1, 0.2, -0.2, 0.5, 0.6, -0.6, -0.3,
     -0.5} // n=13
};

// Pre-computed Schmidt quasi-normalization factors
// S(n,m) = sqrt((2-delta(0,m))(n-m)!/(n+m)!)
float SCHMIDT_FACTORS[MAX_ORDER + 1][MAX_ORDER + 1] = {
    {1.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=0
    {1.000000, 1.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=1
    {1.000000, 0.577350, 0.288675, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=2
    {1.000000, 0.408248, 0.129099, 0.052705, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=3
    {1.000000, 0.316228, 0.074536, 0.019920, 0.007043, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=4
    {1.000000, 0.258199, 0.048795, 0.009960, 0.002348, 0.000742, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=5
    {1.000000, 0.218218, 0.034503, 0.005751, 0.001050, 0.000224, 0.000065,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=6
    {1.000000, 0.188982, 0.025717, 0.003637, 0.000548, 0.000091, 0.000018,
     0.000005, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=7
    {1.000000, 0.166667, 0.019920, 0.002452, 0.000317, 0.000044, 0.000007,
     0.000001, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=8
    {1.000000, 0.149071, 0.015891, 0.001734, 0.000196, 0.000023, 0.000003,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=9
    {1.000000, 0.134840, 0.012975, 0.001272, 0.000129, 0.000014, 0.000002,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=10
    {1.000000, 0.123091, 0.010796, 0.000962, 0.000088, 0.000008, 0.000001,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=11
    {1.000000, 0.113228, 0.009124, 0.000745, 0.000062, 0.000005, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000}, // n=12
    {1.000000, 0.104828, 0.007813, 0.000589, 0.000045, 0.000004, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000} // n=13
};

void compute_B(slate_t *slate)
{
    const float alt = slate->geodetic[0]; // altitude (km)
    const float lat = slate->geodetic[1]; // latitude (-90 to 90)
    const float lon = slate->geodetic[2]; // longitude (-180 to 180)

    // Input validation to prevent NaN
    if (alt < -1000.0f || alt > 10000.0f)
    {
        LOG_ERROR("Invalid altitude: %f km", alt);
        slate->B_est[0] = slate->B_est[1] = slate->B_est[2] = 0.0f;
        return;
    }
    if (lat < -90.0f || lat > 90.0f)
    {
        LOG_ERROR("Invalid latitude: %f degrees", lat);
        slate->B_est[0] = slate->B_est[1] = slate->B_est[2] = 0.0f;
        return;
    }
    if (lon < -180.0f || lon > 180.0f)
    {
        LOG_ERROR("Invalid longitude: %f degrees", lon);
        slate->B_est[0] = slate->B_est[1] = slate->B_est[2] = 0.0f;
        return;
    }

    // Convert to spherical coordinates using math spherical coordinate
    // conventions
    const float theta = (90.0f - lat) * M_PI / 180.0f; // colatitude [0 to π]
    const float phi = (lon + 180.0f) * M_PI / 180.0f;  // azimuth [-π to π]

    float Br = 0.0f;
    float Btheta = 0.0f;
    float Bphi = 0.0f;

    const float r_ratio = R_E / (R_E + alt);

    // Cache trig terms with pole regularization
    const float sin_theta = sin(theta); // for pole regularization
    const float cos_theta = cos(theta); // for Legendre polynomials

    // // Regularization for pole proximity (prevents division by zero)
    const float POLE_THRESH = 1e-7f; // Added missing constant
    const float sin_theta_reg =
        (fabs(sin_theta) < POLE_THRESH)
            ? POLE_THRESH * (sin_theta >= 0.0f ? 1.0f : -1.0f)
            : sin_theta;
    // FLAG: IGNORE TRIADS IN THIS CASE

    // Pre-compute sin/cos m*phi terms
    float sin_mphi[MODEL_ORDER + 1] = {0.0f};
    float cos_mphi[MODEL_ORDER + 1] = {0.0f};
    for (int m = 0; m <= MODEL_ORDER; m++)
    {
        sin_mphi[m] = sin(m * phi);
        cos_mphi[m] = cos(m * phi);
    }

    // Main field computation loop
    float r_ratio_n = r_ratio * r_ratio * r_ratio; // Start at n=1 term T
    for (int n = 1; n <= MODEL_ORDER; n++)
    {
        for (int m = 0; m <= n; m++)
        {
            // Get Schmidt normalized associated Legendre functions
            const float P = legendre_schmidt(n, m, cos_theta);
            const float dP = -sin_theta * d_legendre_schmidt(n, m, cos_theta);

            // Compute common term for efficiency
            const float term = (g[n][m] * cos_mphi[m] + h[n][m] * sin_mphi[m]);

            // Accumulate field components
            // Magnetic field in r direction
            Br += (float)(n + 1) * r_ratio_n * P * term;
            // Magnetic field in latitude direction
            Btheta -= r_ratio_n * dP * term;

            // Handle Bphi carefully near poles
            if (m > 0)
            { // m=0 terms don't contribute to Bphi
                const float Bphi_term =
                    (-g[n][m] * sin_mphi[m] + h[n][m] * cos_mphi[m]) *
                    (float)m * P / sin_theta_reg;
                // (float)m * P / sin_theta_reg *

                // Smooth transition near poles
                const float pole_factor = (fabs(sin_theta) < POLE_THRESH)
                                              ? sin_theta / POLE_THRESH
                                              : 1.0f;

                Bphi -= r_ratio_n * Bphi_term * pole_factor;
            }
        }
        // Multiply another ratio before next loop
        r_ratio_n *= r_ratio;
    }

    // Check for NaN values before storing
    if (std::isnan(Br) || std::isnan(Btheta) || std::isnan(Bphi) ||
        std::isinf(Br) || std::isinf(Btheta) || std::isinf(Bphi))
    {
        LOG_ERROR(
            "NaN/Inf detected in magnetic field: Br=%f, Btheta=%f, Bphi=%f", Br,
            Btheta, Bphi);
        LOG_ERROR("Input coordinates: alt=%f, lat=%f, lon=%f", alt, lat, lon);
        slate->B_est = float3(0.0f, 0.0f, 0.0f);
        return;
    }

    // Store results
    slate->B_est = float3(Br, Bphi, Btheta);
}

// Modified Legendre function using pre-computed factors
float legendre_schmidt(int n, int m, float x)
{
    if (n > MODEL_ORDER || m > n)
        return 0;

    // First compute P_m^m
    float pmm = 1.0f;

    if (m > 0)
    {
        // Clamp x to valid range to prevent NaN from sqrt
        float x_clamped = fmax(-1.0f, fmin(1.0f, x));
        float somx2 = sqrt((1.0f - x_clamped) * (1.0f + x_clamped));
        float fact = 1.0f;
        for (int i = 1; i <= m; i++)
        {
            pmm *= -fact * somx2; // Add negative sign and alternating factor
            fact += 2.0f;         // This grows as 1, 3, 5, 7, ...
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
    float pnm = 0.0f;
    for (int k = m + 2; k <= n; k++)
    {
        pnm = ((2.0f * k - 1.0f) * x * pmmp1 / SCHMIDT_FACTORS[k - 1][m] -
               (k + m - 1.0f) * pmm / SCHMIDT_FACTORS[k - 2][m]) /
              (k - m);
        // Apply Schmidt factor ratio for this k
        pnm *= SCHMIDT_FACTORS[k][m];
        pmm = pmmp1;
        pmmp1 = pnm;
    }

    return pmmp1; // Return pmmp1 for the final value
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
    const float denominator = (x * x) - 1.0f;
    if (fabs(denominator) < 1e-10f)
        return 0.0f; // Handle near-pole case
    const float factor = 1.0f / denominator;

    if (n == m)
    {
        return n * x * legendre_schmidt(n, m, x) * factor;
    }

    // We need both P_n^m and P_(n-1)^m for the derivative
    const float P_n = legendre_schmidt(n, m, x);
    const float P_nm1 = legendre_schmidt(n - 1, m, x);

    return (n * x * P_n - (n + m) * P_nm1) * factor;
}