/**
 * @author Chen Li and Lundeen Cahilly
 * @date 2025-07-04
 */

#include "sun_vector.h"
#include "constants.h"
#include "macros.h"
#include "pico/stdlib.h"
#include "gnc/utils/utils.h"

// Note: the difference between UTC and UT1 is ignored

void compute_sun_vector_eci(slate_t *slate)
{
    int Y = slate->UTC_date[0];     // YEAR at index 0
    int M = slate->UTC_date[1];     // MONTH at index 1
    int D_int = slate->UTC_date[2]; // DAY at index 2

    int hh = static_cast<int>(slate->UTC_time / 10000);
    int mm = static_cast<int>(fmodf(slate->UTC_time, 10000) / 100);
    float ss = fmodf(slate->UTC_time, 100);

    float D = D_int + (hh + mm / 60.0f + ss / 3600.0f) / 24.0f;

    int y, m;
    if (M <= 2)
    {
        y = Y - 1;
        m = M + 12;
    }
    else
    {
        y = Y;
        m = M;
    }

    float B = floorf(y / 400.0f) - floorf(y / 100.0f) + floorf(y / 4.0f);

    float mjd =
        365.0f * y - 679004.0f + floorf(B) + floorf(30.6001f * (m + 1)) + D;

    // Julian centuries since J2000
    float T = (mjd - 51544.0f) / 36525.0f;

    // Mean Longitude [deg]
    float L0 = 280.46646f + 36000.76983f * T + 0.0003032f * T * T;

    // Mean Anomaly [deg]
    float MA = 357.52911f + 35999.05029f * T - 0.0001537f * T * T;
    float M_rad = MA * DEG_TO_RAD;

    // Sun’s Ecliptic Longitude [deg]
    float C = (1.914602f - 0.004817f * T - 0.000014f * T * T) * sin(M_rad) +
              (0.019993f - 0.000101f * T) * sin(2.0f * M_rad) +
              0.000289f * sin(3.0f * M_rad);
    float lambda = L0 + C;
    float lambda_rad = lambda * DEG_TO_RAD;

    // Obliquity of the Ecliptic [deg]
    float epsilon = 23.439291f - 0.0130042f * T;
    float epsilon_rad = epsilon * DEG_TO_RAD;

    // Compute Unit Vector in ECI
    float rX = cos(lambda_rad);
    float rY = cos(epsilon_rad) * sin(lambda_rad);
    float rZ = sin(epsilon_rad) * sin(lambda_rad);

    slate->sun_vector_eci = {rX, rY, rZ};
}
