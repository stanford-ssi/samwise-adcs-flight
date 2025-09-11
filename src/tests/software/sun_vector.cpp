/**
 * @author Chen Li and Lundeen Cahilly
 * @date 2025-09-05
 *
 * Sun vector model tests
 */

#include "sun_vector.h"
#include "gnc/models/sun_vector.h"
#include "macros.h"
#include "pico/stdlib.h"

void test_sun_vector_eci(slate_t *slate)
{
    // Initialize
    slate->UTC_date = {2025, 7, 1};
    slate->UTC_time = 161300.0f;
    slate->sun_vector_eci = {0.0f, 0.0f, 0.0f};

    compute_sun_vector_eci(slate);

    printf("x=", slate->sun_vector_eci[0]);
    printf("y=", slate->sun_vector_eci[1]);
    printf("z=", slate->sun_vector_eci[2]);
}

void test_sun_vector_year(slate_t *slate)
{
    printf("[sun vector test] Calculate sun vector over 1 year\n");
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
        slate->UTC_time = time_seconds;
        slate->sun_vector_eci[0] = 0.0f;
        slate->sun_vector_eci[1] = 0.0f;
        slate->sun_vector_eci[2] = 0.0f;

        // Compute sun vector
        compute_sun_vector_eci(slate);

        // Convert time_seconds to hours, minutes, seconds
        int hours = (int)(time_seconds / 3600.0f);
        int minutes = (int)((time_seconds - hours * 3600.0f) / 60.0f);
        int seconds = (int)(time_seconds - hours * 3600.0f - minutes * 60.0f);

        // Output as CSV format: datetime,x,y,z
        // Using ISO format: YYYY-MM-DD HH:MM:SS
        printf("%04d-%02d-%02d %02d:%02d:%02d,%.6f,%.6f,%.6f\n", (int)year,
               (int)month, (int)day, hours, minutes, seconds,
               slate->sun_vector_eci[0], slate->sun_vector_eci[1],
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