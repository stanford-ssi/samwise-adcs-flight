/**
 * @author Chen Li
 * @date 2025-03-03
 */

#include "sun_vector.h"
#include "constants.h"
#include "macros.h"
#include "pico/stdlib.h"

float wrapTo360(float angle)
{
    return fmodf(fmodf(angle, 360.0f) + 360.0f, 360.0f);
}

// Note: the difference between UTC and UT1 is ignored

void compute_sun_vector_eci(slate_t *slate)
{
    // UTC_date is hard coded for now, current GPS does not provide date
    float year = 2025.0f;
    float month = 9.0f;
    float day = 4.0f;

    slate->UTC_date[0] = year;
    slate->UTC_date[1] = month;
    slate->UTC_date[2] = day;
    // end of hard coded UTC_date

    int Y = slate->UTC_date[0];     // YEAR at index 0
    int M = slate->UTC_date[1];     // MONTH at index 1
    int D_int = slate->UTC_date[2]; // DAY at index 2

    int hh = static_cast<int>(slate->gps_time / 10000);
    int mm = static_cast<int>(fmodf(slate->gps_time, 10000) / 100);
    float ss = fmodf(slate->gps_time, 100);

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

    // slate->sun_vector_eci[0] = X;
    // slate->sun_vector_eci[1] = Y;
    // slate->sun_vector_eci[2] = Z;
    slate->sun_vector_eci = {rX, rY, rZ};
}

void test_sun_vector_eci(slate_t *slate)
{
    // Initialize
    slate->UTC_date = {2025, 7, 1};
    slate->gps_time = 161300.0f;
    slate->sun_vector_eci = {0.0f, 0.0f, 0.0f};

    compute_sun_vector_eci(slate);

    printf("x=", slate->sun_vector_eci[0]);
    printf("y=", slate->sun_vector_eci[1]);
    printf("z=", slate->sun_vector_eci[2]);
}

void test_sun_vector_year(slate_t *slate)
{
    printf("test sun vector for a year\n");
    // Test parameters
    const int delta_hours = 6;    // Sample every 6 hours (change as needed)
    const int days_in_year = 370; // A bit over a year
    const int samples_per_day = 24 / delta_hours;
    const int total_samples = days_in_year * samples_per_day;

    // Starting date: January 1, 2025, 00:00:00
    float year = 2025.0f;
    float month = 1.0f;
    float day = 1.0f;
    float time_seconds = 0.0f;

    // Days per month (non-leap year)
    int days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    // Check for leap year
    int year_int = (int)year;
    if ((year_int % 4 == 0 && year_int % 100 != 0) || (year_int % 400 == 0))
    {
        days_in_month[1] = 29;
    }

    for (int i = 0; i < total_samples; i++)
    {
        // Set current date and time
        slate->UTC_date[0] = year;
        slate->UTC_date[1] = month;
        slate->UTC_date[2] = day;
        slate->gps_time = time_seconds;
        slate->sun_vector_eci[0] = 0.0f;
        slate->sun_vector_eci[1] = 0.0f;
        slate->sun_vector_eci[2] = 0.0f;

        // Compute sun vector
        compute_sun_vector_eci(slate);

        // Output as CSV format: date,x,y,z
        // Using ISO format for date: YYYY-MM-DD
        printf("%04d-%02d-%02d,%.6f,%.6f,%.6f\n", (int)year, (int)month,
               (int)day, slate->sun_vector_eci[0], slate->sun_vector_eci[1],
               slate->sun_vector_eci[2]);

        // Increment time
        time_seconds += delta_hours * 3600.0f;

        // Handle day rollover
        if (time_seconds >= 86400.0f)
        {
            time_seconds -= 86400.0f;
            day += 1.0f;

            // Handle month rollover
            if (day > days_in_month[(int)month - 1])
            {
                day = 1.0f;
                month += 1.0f;

                // Handle year rollover
                if (month > 12.0f)
                {
                    month = 1.0f;
                    year += 1.0f;
                }
            }
        }
    }
}