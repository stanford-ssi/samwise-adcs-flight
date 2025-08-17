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
#define MODEL_ORDER 13

// Forward declare legendre polynomial functions
void compute_legendre_polynomials(int nmax, float theta,
                                  float Pnm[][MAX_ORDER + 2]);

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

// Dynamic Schmidt normalization - computed at runtime like pyigrf
float rootn[2 * MAX_ORDER * MAX_ORDER + 1];

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
    const float phi = (lon)*M_PI / 180.0f;             // azimuth [-π to π]

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

    // Compute Legendre polynomials using pyigrf strategy
    float Pnm[MAX_ORDER + 1][MAX_ORDER + 2];
    compute_legendre_polynomials(MODEL_ORDER, theta, Pnm);

    // Main field computation loop
    float r_ratio_n = r_ratio * r_ratio * r_ratio; // Start at n=1 term
    for (int n = 1; n <= MODEL_ORDER; n++)
    {
        for (int m = 0; m <= n; m++)
        {
            // Get Schmidt normalized associated Legendre functions and
            // derivatives
            const float P = Pnm[n][m];
            const float dP =
                Pnm[m][n + 1]; // pyigrf stores derivatives at [m][n+1]

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

// pyigrf-style Legendre polynomial computation
void compute_legendre_polynomials(int nmax, float theta,
                                  float Pnm[][MAX_ORDER + 2])
{
    // Initialize rootn array like pyigrf
    for (int i = 0; i <= 2 * nmax * nmax; i++)
    {
        rootn[i] = sqrt((float)i);
    }

    float costh = cos(theta);
    float sinth = sqrt(1.0f - costh * costh);

    // Initialize arrays
    for (int n = 0; n <= nmax; n++)
    {
        for (int m = 0; m <= nmax + 1; m++)
        {
            Pnm[n][m] = 0.0f;
        }
    }

    Pnm[0][0] = 1.0f;  // P(0,0) = 1
    Pnm[1][1] = sinth; // P(1,1) = sin(theta)

    // Recursion relations after Langel "The Main Field" (1987)
    for (int m = 0; m < nmax; m++)
    {
        float Pnm_tmp = rootn[m + m + 1] * Pnm[m][m];
        Pnm[m + 1][m] = costh * Pnm_tmp;

        if (m > 0)
        {
            Pnm[m + 1][m + 1] = sinth * Pnm_tmp / rootn[m + m + 2];
        }

        for (int n = m + 2; n <= nmax; n++)
        {
            int d = n * n - m * m;
            int e = n + n - 1;
            Pnm[n][m] = ((float)e * costh * Pnm[n - 1][m] -
                         rootn[d - e] * Pnm[n - 2][m]) /
                        rootn[d];
        }
    }

    // Compute derivatives: dP(n,m) = Pnm[m][n+1]
    Pnm[0][2] = -Pnm[1][1];
    Pnm[1][2] = Pnm[1][0];

    for (int n = 2; n <= nmax; n++)
    {
        Pnm[0][n + 1] = -sqrt((float)(n * n + n) / 2.0f) * Pnm[n][1];
        Pnm[1][n + 1] = (sqrt(2.0f * (float)(n * n + n)) * Pnm[n][0] -
                         sqrt((float)(n * n + n - 2)) * Pnm[n][2]) /
                        2.0f;

        for (int m = 2; m < n; m++)
        {
            Pnm[m][n + 1] =
                0.5f *
                (sqrt((float)(n + m) * (float)(n - m + 1)) * Pnm[n][m - 1] -
                 sqrt((float)(n + m + 1) * (float)(n - m)) * Pnm[n][m + 1]);
        }

        Pnm[n][n + 1] = sqrt(2.0f * (float)n) * Pnm[n][n - 1] / 2.0f;
    }
}