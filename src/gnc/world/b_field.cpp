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

// Pre-computed square root lookup table for Schmidt normalization
constexpr float
    sqrt_lut[2 * B_FIELD_MODEL_MAX_ORDER * B_FIELD_MODEL_MAX_ORDER + 1] = {
        0.0000000000f,  1.0000000000f,  1.4142135624f,  1.7320508076f,
        2.0000000000f,  2.2360679775f,  2.4494897428f,  2.6457513111f,
        2.8284271247f,  3.0000000000f,  3.1622776602f,  3.3166247904f,
        3.4641016151f,  3.6055512755f,  3.7416573868f,  3.8729833462f,
        4.0000000000f,  4.1231056256f,  4.2426406871f,  4.3588989435f,
        4.4721359550f,  4.5825756950f,  4.6904157598f,  4.7958315233f,
        4.8989794856f,  5.0000000000f,  5.0990195136f,  5.1961524227f,
        5.2915026221f,  5.3851648071f,  5.4772255751f,  5.5677643628f,
        5.6568542495f,  5.7445626465f,  5.8309518948f,  5.9160797831f,
        6.0000000000f,  6.0827625303f,  6.1644140030f,  6.2449979984f,
        6.3245553203f,  6.4031242374f,  6.4807406984f,  6.5574385243f,
        6.6332495807f,  6.7082039325f,  6.7823299831f,  6.8556546004f,
        6.9282032303f,  7.0000000000f,  7.0710678119f,  7.1414284285f,
        7.2111025509f,  7.2801098893f,  7.3484692283f,  7.4161984871f,
        7.4833147735f,  7.5498344353f,  7.6157731059f,  7.6811457479f,
        7.7459666924f,  7.8102496759f,  7.8740078740f,  7.9372539332f,
        8.0000000000f,  8.0622577483f,  8.1240384046f,  8.1853527719f,
        8.2462112512f,  8.3066238629f,  8.3666002653f,  8.4261497732f,
        8.4852813742f,  8.5440037453f,  8.6023252670f,  8.6602540378f,
        8.7177978871f,  8.7749643874f,  8.8317608663f,  8.8881944173f,
        8.9442719100f,  9.0000000000f,  9.0553851381f,  9.1104335791f,
        9.1651513899f,  9.2195444573f,  9.2736184955f,  9.3273790531f,
        9.3808315196f,  9.4339811321f,  9.4868329805f,  9.5393920142f,
        9.5916630466f,  9.6436507610f,  9.6953597148f,  9.7467943448f,
        9.7979589711f,  9.8488578018f,  9.8994949366f,  9.9498743711f,
        10.0000000000f, 10.0498756211f, 10.0995049384f, 10.1488915651f,
        10.1980390272f, 10.2469507660f, 10.2956301410f, 10.3440804328f,
        10.3923048454f, 10.4403065089f, 10.4880884817f, 10.5356537529f,
        10.5830052443f, 10.6301458127f, 10.6770782520f, 10.7238052948f,
        10.7703296143f, 10.8166538264f, 10.8627804912f, 10.9087121146f,
        10.9544511501f, 11.0000000000f, 11.0453610172f, 11.0905365064f,
        11.1355287257f, 11.1803398875f, 11.2249721603f, 11.2694276696f,
        11.3137084990f, 11.3578166916f, 11.4017542510f, 11.4455231423f,
        11.4891252931f, 11.5325625947f, 11.5758369028f, 11.6189500386f,
        11.6619037897f, 11.7046999107f, 11.7473401245f, 11.7898261226f,
        11.8321595662f, 11.8743420870f, 11.9163752878f, 11.9582607431f,
        12.0000000000f, 12.0415945788f, 12.0830459736f, 12.1243556530f,
        12.1655250606f, 12.2065556157f, 12.2474487139f, 12.2882057274f,
        12.3288280059f, 12.3693168769f, 12.4096736460f, 12.4498995980f,
        12.4899959968f, 12.5299640861f, 12.5698050900f, 12.6095202129f,
        12.6491106407f, 12.6885775404f, 12.7279220614f, 12.7671453348f,
        12.8062484749f, 12.8452325787f, 12.8840987267f, 12.9228479833f,
        12.9614813968f, 13.0000000000f, 13.0384048104f, 13.0766968306f,
        13.1148770486f, 13.1529464380f, 13.1909059583f, 13.2287565553f,
        13.2664991614f, 13.3041346957f, 13.3416640641f, 13.3790881603f,
        13.4164078650f, 13.4536240471f, 13.4907375632f, 13.5277492585f,
        13.5646599663f, 13.6014705087f, 13.6381816970f, 13.6747943312f,
        13.7113092008f, 13.7477270849f, 13.7840487521f, 13.8202749611f,
        13.8564064606f, 13.8924439894f, 13.9283882772f, 13.9642400438f,
        14.0000000000f, 14.0356688476f, 14.0712472795f, 14.1067359797f,
        14.1421356237f, 14.1774468788f, 14.2126704036f, 14.2478068488f,
        14.2828568571f, 14.3178210633f, 14.3527000944f, 14.3874945699f,
        14.4222051019f, 14.4568322948f, 14.4913767462f, 14.5258390463f,
        14.5602197786f, 14.5945195193f, 14.6287388383f, 14.6628782986f,
        14.6969384567f, 14.7309198627f, 14.7648230602f, 14.7986485869f,
        14.8323969742f, 14.8660687473f, 14.8996644258f, 14.9331845231f,
        14.9666295471f, 15.0000000000f, 15.0332963784f, 15.0665191733f,
        15.0996688705f, 15.1327459504f, 15.1657508881f, 15.1986841536f,
        15.2315462117f, 15.2643375225f, 15.2970585408f, 15.3297097168f,
        15.3622914957f, 15.3948043183f, 15.4272486205f, 15.4596248337f,
        15.4919333848f, 15.5241746963f, 15.5563491861f, 15.5884572681f,
        15.6204993518f, 15.6524758425f, 15.6843871414f, 15.7162336455f,
        15.7480157480f, 15.7797338381f, 15.8113883008f, 15.8429795178f,
        15.8745078664f, 15.9059737206f, 15.9373774505f, 15.9687194227f,
        16.0000000000f, 16.0312195419f, 16.0623784042f, 16.0934769394f,
        16.1245154966f, 16.1554944214f, 16.1864140562f, 16.2172747402f,
        16.2480768093f, 16.2788205961f, 16.3095064303f, 16.3401346384f,
        16.3707055437f, 16.4012194669f, 16.4316767252f, 16.4620776332f,
        16.4924225025f, 16.5227116419f, 16.5529453572f, 16.5831239518f,
        16.6132477258f, 16.6433169771f, 16.6733320005f, 16.7032930885f,
        16.7332005307f, 16.7630546142f, 16.7928556237f, 16.8226038413f,
        16.8522995464f, 16.8819430161f, 16.9115345253f, 16.9410743461f,
        16.9705627485f, 17.0000000000f, 17.0293863659f, 17.0587221092f,
        17.0880074906f, 17.1172427686f, 17.1464281995f, 17.1755640373f,
        17.2046505341f, 17.2336879396f, 17.2626765016f, 17.2916164658f,
        17.3205080757f, 17.3493515729f, 17.3781471970f, 17.4068951855f,
        17.4355957742f, 17.4642491966f, 17.4928556845f, 17.5214154679f,
        17.5499287748f, 17.5783958312f, 17.6068168617f, 17.6351920885f,
        17.6635217327f, 17.6918060130f, 17.7200451467f, 17.7482393493f,
        17.7763888346f, 17.8044938148f, 17.8325545001f, 17.8605710995f,
        17.8885438200f, 17.9164728672f, 17.9443584449f, 17.9722007556f,
        18.0000000000f, 18.0277563773f, 18.0554700853f, 18.0831413200f,
        18.1107702763f, 18.1383571472f, 18.1659021246f, 18.1934053987f,
        18.2208671583f, 18.2482875909f, 18.2756668825f, 18.3030052177f,
        18.3303027798f, 18.3575597507f, 18.3847763109f};

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