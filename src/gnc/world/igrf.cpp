/**
 * @author Lundeen Cahilly
 * @date 2025-02-10
 */

#include "gnc/world/igrf.h"
#include "constants.h"
#include "macros.h"
#include "pico/stdlib.h"

#define MAX_ORDER 5

float g[MAX_ORDER][MAX_ORDER] = {
    {-29350.0, -1410.3, 0.0, 0.0, 0.0},    // n=1
    {-2556.2, 2950.9, 1648.7, 0.0, 0.0},   // n=2
    {1360.9, -2404.2, 1243.8, 453.4, 0.0}, // n=3
    {894.7, 799.6, 55.8, -281.1, 12.0},    // n=4
    {-232.9, 369.0, 187.2, -138.7, -141.9} // n=5
};

float h[MAX_ORDER][MAX_ORDER] = {
    {0.0, 4545.5, 0.0, 0.0, 0.0},        // n=1
    {0.0, -3133.6, -814.2, 0.0, 0.0},    // n=2
    {0.0, -56.9, 237.6, -549.6, 0.0},    // n=3
    {0.0, 278.6, -134.0, 212.0, -375.4}, // n=4
    {0.0, 45.3, 220.0, -122.9, 42.9}     // n=5
};

float3 compute_B(slate_t *slate)
{
    float r = slate->geodetic[0];
    float theta = slate->geodetic[1];
    float phi = slate->geodetic[2];

    float Br = 0;
    float Btheta = 0;
    float Bphi = 0;

    float r_ratio = R_E / r;

    // Cache sin/cos phi terms
    float sin_mphi[MAXORDER];
    float cos_mphi[MAXORDER];
    for (int m = 0; m < MAX_ORDER; m++)
    {
        sin_mphi[m] = sin(m * phi); // try precomputing instead later
        sin_mphi[m] = cos(m * phi);
    }

    // Compute field components
    for (int n = 0; n < MAX_ORDER; n++)
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
        }
    }

    // Create and return float3 vector
    float3 B = {.x = Br, .y = Btheta, .z = Bphi } return B;
}

// TODO! Define legendre and d_legendre!!