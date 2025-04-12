/**
 * @author Chen Li
 * @date 2025-03-03
 */

#include "gnc/sun_vector.h"
#include "macros.h"
#include "pico/stdlib.h"

constexpr float DEG2RAD =
    3.14159f / 180.0f; // do we have math? Using numbers for pi for now

void compute_sun_vector_eci(slate_t *slate)
{
    // Convert MJD to Julian Date
    float JD = slate->MJD + 2400000.5f;

    // Julian centuries since J2000
    float T = (JD - 2451545.0f) / 36525.0f;

    // Mean Longitude [deg]
    float L0 = 280.46646f + 36000.76983f * T + 0.0003032f * T * T;

    // Mean Anomaly [deg]
    float M = 357.52911f + 35999.05029f * T - 0.0001537f * T * T;
    float M_rad = M * DEG2RAD;

    // Sun’s Ecliptic Longitude [deg]
    float C = (1.914602f - 0.004817f * T - 0.000014f * T * T) * sin(M_rad) +
              (0.019993f - 0.000101f * T) * sin(2.0f * M_rad) +
              0.000289f * sin(3.0f * M_rad);
    float lambda = L0 + C;
    float lambda_rad = lambda * DEG2RAD;

    // Obliquity of the Ecliptic [deg]
    float epsilon = 23.439291f - 0.0130042f * T;
    float epsilon_rad = epsilon * DEG2RAD;

    // Compute Unit Vector in ECI
    float X = cos(lambda_rad);
    float Y = cos(epsilon_rad) * sin(lambda_rad);
    float Z = sin(epsilon_rad) * sin(lambda_rad);

    // slate->sun_vector_eci[0] = X;
    // slate->sun_vector_eci[1] = Y;
    // slate->sun_vector_eci[2] = Z;
    slate->sun_vector_eci = {X, Y, Z};
}

void test_sun_vector_eci(slate_t *slate)
{
    // Initialize
    slate->MJD = 51544.50000000f;
    slate->sun_vector_eci = {0.0f, 0.0f, 0.0f};

    compute_sun_vector_eci(slate);

    printf("x=", slate->sun_vector_eci[0]);
    printf("y=", slate->sun_vector_eci[1]);
    printf("z=", slate->sun_vector_eci[2]);
}