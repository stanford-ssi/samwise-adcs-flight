/**
 * @author Lundeen Cahilly
 * @date 2025-02-10
 *
 * This file defines a magnetic field model based on the IGRF-2025 coefficients.
 * It computes the magnetic field vector in based on the satellite's geodetic
 * coordinates (altitude, latitude, longitude).
 */

#include "b_field.h"

#include "pico/stdlib.h"
#include <cmath>

#include "../frame_transforms.h"
#include "constants.h"
#include "linalg.h"
#include "macros.h"

// Forward declaration
void compute_legendre_polynomials(int nmax, float theta,
                                  float Pnm[][B_FIELD_MODEL_MAX_ORDER + 2],
                                  float cos_theta, float sin_theta);

// IGRF 2025 Coefficients (up to n=13) [nT]
// Indices are n going down and m going left to right
// There is no n=0 term (would imply a magnetic monopole)
constexpr float g[B_FIELD_MODEL_MAX_ORDER + 1][B_FIELD_MODEL_MAX_ORDER + 1] = {
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
constexpr float h[B_FIELD_MODEL_MAX_ORDER + 1][B_FIELD_MODEL_MAX_ORDER + 1] = {
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
float rootn[2 * B_FIELD_MODEL_MAX_ORDER * B_FIELD_MODEL_MAX_ORDER + 1];

bool compute_B(slate_t *slate)
{
    const float lat = slate->lla[0]; // latitude (-90 to 90)
    const float lon = slate->lla[1]; // longitude (-180 to 180)
    const float alt = slate->lla[2]; // altitude (km)

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

    // Convert to spherical coordinates using math spherical coordinate
    // conventions
    const float theta = (90.0f - lat) * DEG_TO_RAD; // colatitude [0 to π]
    const float phi = lon * DEG_TO_RAD;             // azimuth [-π to π]

    float Br = 0.0f;
    float Btheta = 0.0f;
    float Bphi = 0.0f;

    const float r_ratio = R_E / (R_E + alt);

    // Cache trig terms with pole regularization
    const float sin_theta = sinf(theta); // for pole regularization
    const float cos_theta = cosf(theta); // for Legendre polynomials
    const float sin_theta_reg =
        (fabsf(sin_theta) < B_FIELD_POLE_THRESH)
            ? B_FIELD_POLE_THRESH * (sin_theta >= 0.0f ? 1.0f : -1.0f)
            : sin_theta;
    // FLAG: IGNORE TRIADS IN THIS CASE

    // Pre-compute sin/cos m*phi terms
    float sin_mphi[B_FIELD_MODEL_ORDER + 1];
    float cos_mphi[B_FIELD_MODEL_ORDER + 1];
    for (int m = 0; m <= B_FIELD_MODEL_ORDER; m++)
    {
        sin_mphi[m] = sinf(m * phi);
        cos_mphi[m] = cosf(m * phi);
    }

    // Compute Legendre polynomials using pyigrf strategy
    float Pnm[B_FIELD_MODEL_MAX_ORDER + 1][B_FIELD_MODEL_MAX_ORDER + 2];
    compute_legendre_polynomials(B_FIELD_MODEL_ORDER, theta, Pnm, cos_theta,
                                 sin_theta);

    // Main field computation loop
    float r_ratio_n = r_ratio * r_ratio * r_ratio; // Start at n=1 term
    for (int n = 1; n <= B_FIELD_MODEL_ORDER; n++)
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
            Br += static_cast<float>(n + 1) * r_ratio_n * P * term;
            // Magnetic field in latitude direction
            Btheta -= r_ratio_n * dP * term;

            // Handle Bphi carefully near poles
            if (m > 0)
            { // m=0 terms don't contribute to Bphi
                const float Bphi_term =
                    (-g[n][m] * sin_mphi[m] + h[n][m] * cos_mphi[m]) *
                    static_cast<float>(m) * P / sin_theta_reg;

                // Smooth transition near poles
                const float pole_factor =
                    (fabsf(sin_theta) < B_FIELD_POLE_THRESH)
                        ? sin_theta / B_FIELD_POLE_THRESH
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
        return false;
    }

    // Store results in r, phi, theta frame (for graphing) [Up, East, North]
    slate->B_est_rpt = float3(Br, Bphi, Btheta);

    // Store results in East-North-Up (ENU) frame
    slate->B_est_enu = float3(Bphi, Btheta, Br);

    // Convert to ECEF frame and store
    slate->B_est_ecef =
        enu_to_ecef(float3(Bphi, Btheta, Br), float3(lat, lon, alt));

    return true;
}

// pyigrf-style Legendre polynomial computation
void compute_legendre_polynomials(int nmax, float theta,
                                  float Pnm[][B_FIELD_MODEL_MAX_ORDER + 2],
                                  float cos_theta, float sin_theta)
{
    // Initialize rootn array like pyigrf
    for (int i = 0; i <= 2 * nmax * nmax; i++)
    {
        rootn[i] = sqrtf(static_cast<float>(i));
    }

    // Initialize arrays
    for (int n = 0; n <= nmax; n++)
    {
        for (int m = 0; m <= nmax + 1; m++)
        {
            Pnm[n][m] = 0.0f;
        }
    }

    Pnm[0][0] = 1.0f;      // P(0,0) = 1
    Pnm[1][1] = sin_theta; // P(1,1) = sin(theta)

    // Recursion relations after Langel "The Main Field" (1987)
    for (int m = 0; m < nmax; m++)
    {
        float Pnm_tmp = rootn[m + m + 1] * Pnm[m][m];
        Pnm[m + 1][m] = cos_theta * Pnm_tmp;

        if (m > 0)
        {
            Pnm[m + 1][m + 1] = sin_theta * Pnm_tmp / rootn[m + m + 2];
        }

        for (int n = m + 2; n <= nmax; n++)
        {
            int d = n * n - m * m;
            int e = n + n - 1;
            Pnm[n][m] = (static_cast<float>(e) * cos_theta * Pnm[n - 1][m] -
                         rootn[d - e] * Pnm[n - 2][m]) /
                        rootn[d];
        }
    }

    // Compute derivatives: dP(n,m) = Pnm[m][n+1]
    Pnm[0][2] = -Pnm[1][1];
    Pnm[1][2] = Pnm[1][0];

    for (int n = 2; n <= nmax; n++)
    {
        Pnm[0][n + 1] =
            -sqrtf(static_cast<float>(n * n + n) / 2.0f) * Pnm[n][1];
        Pnm[1][n + 1] =
            (sqrtf(2.0f * static_cast<float>(n * n + n)) * Pnm[n][0] -
             sqrtf(static_cast<float>(n * n + n - 2)) * Pnm[n][2]) /
            2.0f;

        for (int m = 2; m < n; m++)
        {
            Pnm[m][n + 1] = 0.5f * (sqrtf(static_cast<float>(n + m) *
                                          static_cast<float>(n - m + 1)) *
                                        Pnm[n][m - 1] -
                                    sqrtf(static_cast<float>(n + m + 1) *
                                          static_cast<float>(n - m)) *
                                        Pnm[n][m + 1]);
        }

        Pnm[n][n + 1] =
            sqrtf(2.0f * static_cast<float>(n)) * Pnm[n][n - 1] / 2.0f;
    }
}