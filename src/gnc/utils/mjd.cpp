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
#include "slate.h"

// MJD calculation constants
static const float MJD_EPOCH_OFFSET =
    679004.0f; // Offset to convert from Julian to Modified Julian Date
static const float DAYS_PER_MONTH_COEFFICIENT =
    30.6001f; // Coefficient for month-to-day conversion

// Runs in sensors task after GPS data is read
void compute_MJD(slate_t *slate)
{
    int Y = slate->UTC_date[0];     // YEAR at index 0
    int M = slate->UTC_date[1];     // MONTH at index 1
    int D_int = slate->UTC_date[2]; // DAY at index 2

    // GPS time format is HHMMSS (e.g., 210230 = 21:02:30)
    int hh = static_cast<int>(slate->gps_time / 10000);
    int mm =
        static_cast<int>((static_cast<int>(slate->gps_time) % 10000) / 100);
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

    slate->MJD = 365.0f * y - MJD_EPOCH_OFFSET + floorf(B) +
                 floorf(DAYS_PER_MONTH_COEFFICIENT * (m + 1)) + D;
}