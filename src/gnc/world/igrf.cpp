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

static float legendre(int n, int m, float x)
{
    float pmm = 1.0;

    if (m > 0)
    {
        float somx2 = sqrt((1.0 - x) * (1.0 + x));
        float fact = 1.0;
        for (int i = 1; i <= m; i++)
        {
            pmm *= -fact * somx2;
            fact += 2.0;
        }
    }

    if (n == m)
    {
        return pmm;
    }

    float pmmp1 = x * (2.0 * m + 1.0) * pmm;
    if (n == m + 1)
    {
        return pmmp1;
    }

    float pnm = 0.0;
    for (int k = m + 2; k <= n; k++)
    {
        pnm = (x * (2.0 * k - 1.0) * pmmp1 - (k + m - 1.0) * pmm) / (k - m);
        pmm = pmmp1;
        pmmp1 = pnm;
    }

    return pnm;
}

static float d_legendre(int n, int m, float x)
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