/**
 * @author Lundeen Cahilly
 * @date 2025-10-18
 *
 * GPS and world model task. Reads GPS at 1 Hz and updates reference vectors
 * (b_eci, sun_vector_eci) and position (r_eci, r_ecef) when GPS data is valid.
 */

#include "gps_world_task.h"
#include "constants.h"
#include "macros.h"

#include "drivers/gps/gps.h"
#include "gnc/utils/mjd.h"
#include "gnc/utils/transforms.h"
#include "gnc/world/b_field.h"
#include "gnc/world/sun_vector.h"

/**
 * @brief Initialize GPS and world model task.
 *
 * @param slate Pointer to the current satellite slate
 */
void gps_world_task_init(slate_t *slate)
{
    LOG_INFO("[sensor] Initializing GPS...");

    // --- GPS --- //
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
 * @brief Dispatch GPS and world model task.
 *
 * Reads GPS and updates position, time, and reference vectors (b_eci,
 * sun_vector_eci). Reference vectors only update when GPS provides valid
 * position/time data.
 *
 * @param slate Pointer to the current satellite slate
 */
void gps_world_task_dispatch(slate_t *slate)
{
    if (!slate->gps_alive)
    {
        LOG_DEBUG("[sensor] Skipping GPS due to invalid initialization!");
        return;
    }

    gps_data_t gps_data;
    bool result =
        gps_get_data(&gps_data); // Only returns true if valid data AND fix

    // Change result to false if the timestamp is expired

    if (result)
    {
        slate->gps_lat = gps_data.latitude;
        slate->gps_lon = gps_data.longitude;
        slate->gps_alt = gps_data.altitude / 1000.0f; // Convert m to km
        slate->gps_time =
            static_cast<float>(gps_data.timestamp); // Convert HHMMSS to float

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

        // Compute reference vectors b_eci and sun_vector_eci from GPS
        // position/time
        compute_MJD(slate);
        compute_sun_vector_eci(slate);
        compute_B(slate);

        LOG_DEBUG("[sensor] Lat = %.6f, Lon = %.6f, Alt = %.3f, "
                  "Time = %.3f, "
                  "Date = %02d/%02d/%04d,"
                  "MJD = %.5f",
                  slate->gps_lat, slate->gps_lon, slate->gps_alt,
                  slate->gps_time, day, month, year, slate->MJD);
        LOG_DEBUG("[sensor] sun_vector_eci = [%.6f, %.6f, %.6f]",
                  slate->sun_vector_eci.x, slate->sun_vector_eci.y,
                  slate->sun_vector_eci.z);
        LOG_DEBUG("[sensor] b_eci = [%.3f, %.3f, %.3f]", slate->b_eci.x,
                  slate->b_eci.y, slate->b_eci.z);
    }

    slate->gps_data_valid = result;
}

sched_task_t gps_world_task = {.name = "gps_world",
                               .dispatch_period_ms = 1000, // 1 Hz
                               .task_init = &gps_world_task_init,
                               .task_dispatch = &gps_world_task_dispatch,
                               .next_dispatch = 0};
