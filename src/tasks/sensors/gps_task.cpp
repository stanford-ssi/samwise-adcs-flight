/**
 * @author Lundeen Cahilly
 * @date 2025-10-18
 *
 * GPS task. Reads GPS at 5 Hz and updates position/time data when valid.
 */

#include "gps_task.h"
#include "params.h"
#include "macros.h"

#include "drivers/gps/gps.h"
#include "gnc/estimation/orbit_filter.h"
#include "gnc/utils/mjd.h"
#include "pico/time.h"

/**
 * @brief Initialize GPS task.
 *
 * @param slate Pointer to the current satellite slate
 */
void gps_task_init(slate_t *slate)
{
    LOG_INFO("[sensor] Initializing GPS...");

    bool gps_result = gps_init();
    slate->gps_alive = gps_result;

    if (!gps_result)
    {
        LOG_ERROR("[sensor] Error initializing GPS - deactivating!");
    }
    slate->gps_data_valid = false;

    LOG_INFO("[sensor] GPS Initialization Complete! GPS alive: %s",
             slate->gps_alive ? "true" : "false");
}

/**
 * @brief Dispatch GPS task.
 *
 * Reads GPS and updates position, time, and MJD when GPS provides valid data.
 *
 * @param slate Pointer to the current satellite slate
 */
void gps_task_dispatch(slate_t *slate)
{
    LOG_DEBUG("[sensor] GPS world task dispatching...");
    if (!slate->gps_alive)
    {
        LOG_DEBUG("[sensor] Skipping GPS due to invalid initialization!");
        return;
    }

    gps_data_t gps_data;
    bool data_received =
        gps_get_data(&gps_data); // Only returns true if valid data AND fix

    if (data_received)
    {
        slate->gps_read_time = get_absolute_time();

        slate->gps_lat = gps_data.latitude;
        slate->gps_lon = gps_data.longitude;
        slate->gps_alt = gps_data.altitude / 1000.0f; // Convert m to km
        slate->gps_time =
            static_cast<float>(gps_data.timestamp); // Convert HHMMSS to float
        slate->gps_speed = gps_data.speed;          // knots
        slate->gps_course = gps_data.course;        // degrees true

        // Parse GPS date (DDMMYY format) to UTC_date (year, month, day)
        uint32_t date = gps_data.date;
        uint32_t day = date / 10000;
        uint32_t month = (date / 100) % 100;
        uint32_t year = date % 100;

        // Convert 2-digit year to 4-digit (assume 21st century)
        year += 2000;

        slate->UTC_date[0] = static_cast<float>(year);  // Year
        slate->UTC_date[1] = static_cast<float>(month); // Month
        slate->UTC_date[2] = static_cast<float>(day);   // Day

        // Compute MJD from GPS time/date
        compute_MJD(slate);

        LOG_DEBUG("[sensor] Lat = %.6f, Lon = %.6f, Alt = %.3f, "
                  "Time = %.3f, "
                  "Date = %02d/%02d/%04d, "
                  "MJD = %.5f",
                  slate->gps_lat, slate->gps_lon, slate->gps_alt,
                  slate->gps_time, day, month, year, slate->MJD);

        // Update orbit filter with new GPS measurement
        if (slate->of_is_initialized)
        {
            orbit_filter_update(slate);
        }

        slate->gps_data_valid = true;
    }

    // GPS data stays valid within a time frame, but becomes "stale" outside of
    // it
    else if (slate->gps_data_valid &&
             (get_absolute_time() - slate->gps_read_time >
              GPS_DATA_EXPIRATION_MS * 1000))
    {
        slate->gps_data_valid = false;
    }
}

sched_task_t gps_task = {.name = "gps",
                         .dispatch_period_ms = 200, // 5 Hz
                         .task_init = &gps_task_init,
                         .task_dispatch = &gps_task_dispatch,
                         .next_dispatch = 0};
