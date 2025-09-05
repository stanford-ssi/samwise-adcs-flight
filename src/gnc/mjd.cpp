/**
 * @author Chen Li
 * @date 2025-09-05
 *
 * This file calculates Modified Julian Date based on GPS time, difference
 * between UTC and UT1 is ignored
 */

#include "mjd.h"
#include "macros.h"
#include "pico/printf.h"

// Move MJD calculation from gnc/sun_vector.cpp to here
// It is more clear to calcualte MJD everytime we heard from GPS

// UTC_date is hard coded for now, current GPS does not provide date
void compute_MJD(slate_t *slate)
{
    float year = 2025.0f;
    float month = 9.0f;
    float day = 5.0f;

    slate->UTC_date[0] = year;
    slate->UTC_date[1] = month;
    slate->UTC_date[2] = day;
    // end of hard coded UTC_date

    int Y = slate->UTC_date[0];     // YEAR at index 0
    int M = slate->UTC_date[1];     // MONTH at index 1
    int D_int = slate->UTC_date[2]; // DAY at index 2

    // TODO: check slate->gps_time format
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

    slate->MJD =
        365.0f * y - 679004.0f + floorf(B) + floorf(30.6001f * (m + 1)) + D;
}