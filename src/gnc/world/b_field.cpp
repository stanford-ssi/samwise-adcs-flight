/**
 * @author Lundeen Cahilly
 * @date 2025-02-10
 *
 * This file defines a magnetic field model based on the IGRF-2025 coefficients.
 * It computes the magnetic field vector in based on the satellite's geodetic
 * coordinates (altitude, latitude, longitude).
 */

#include "b_field.h"
#include "b_field_constants.h"

#include "pico/stdlib.h"
#include <cmath>

#include "../frame_transforms.h"
#include "constants.h"
#include "linalg.h"
#include "macros.h"

void compute_legendre_polynomials(int nmax, float theta,
                                  float Pnm[][B_FIELD_MODEL_MAX_ORDER + 2],
                                  float cos_theta, float sin_theta);

// Inline helper function for readable square root lookup
inline float sqrt_lut_get(int index)
{
    return sqrt_lut[index];
}

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