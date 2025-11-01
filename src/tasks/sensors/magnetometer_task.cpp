/**
 * @author Lundeen Cahilly
 * @date 2025-10-18
 *
 * Magnetometer task with non-blocking state machine for handling
 * magnetorquer interference. Task dispatches at 50 Hz to manage timing,
 * but magnetometer data updates at 10 Hz (20ms settle time gives 80%
 * magnetorquer duty cycle).
 */

#include "magnetometer_task.h"
#include "constants.h"
#include "macros.h"

#include "drivers/magnetometer/magnetometer.h"
#include "drivers/magnetorquers/magnetorquers.h"
#include "gnc/estimation/attitude_filter.h"
#include "pico/time.h"

/**
 * State machine for non-blocking magnetometer reads
 */
typedef enum
{
    MAG_IDLE,               // Ready to start a new read
    MAG_WAITING_FOR_SETTLE, // Torquers off, waiting for field to settle
    MAG_READING,            // Reading magnetometer
    MAG_RESTARTING_TORQUERS // Restarting magnetorquers after read
} mag_read_state_t;

static mag_read_state_t mag_state = MAG_IDLE;
static absolute_time_t mag_torquer_off_time;
static absolute_time_t last_mag_read_start = {
    0}; // Track last read cycle start for 100ms period

/**
 * @brief Initialize magnetometer task.
 *
 * @param slate Pointer to the current satellite slate
 */
void magnetometer_task_init(slate_t *slate)
{
    LOG_INFO("[sensor] Initializing magnetometer...");
    rm3100_error_t magnetometer_result = rm3100_init();
    slate->magnetometer_alive = (magnetometer_result == RM3100_OK);

    if (magnetometer_result != RM3100_OK)
    {
        LOG_ERROR("[sensor] Error initializing magnetometer - deactivating!");
    }

    slate->magnetometer_data_valid = false;

    // Initialize state machine
    mag_state = MAG_IDLE;
    last_mag_read_start = get_absolute_time();

    LOG_INFO(
        "[sensor] Magnetometer Initialization Complete! Magnetometer alive: %s",
        slate->magnetometer_alive ? "true" : "false");
}

/**
 * @brief Dispatch magnetometer task. Non-blocking state machine for mag reads.
 *
 * Task runs at 50 Hz for timing management, but magnetometer data only updates
 * at 10 Hz (one full cycle takes ~5 dispatches). State machine ensures
 * magnetorquers are off for MAGNETOMETER_FIELD_SETTLE_TIME_MS before reading,
 * achieving 80% magnetorquer duty cycle (20ms off per 100ms).
 *
 * Tracks last read start time to maintain 100ms read period even if scheduler
 * experiences delays or overruns.
 *
 * @param slate Pointer to the current satellite slate
 */
void magnetometer_task_dispatch(slate_t *slate)
{
    if (!slate->magnetometer_alive)
    {
        return;
    }

    switch (mag_state)
    {
        case MAG_IDLE:
            // Check if 100ms has elapsed since last read cycle started
            if (absolute_time_diff_us(last_mag_read_start,
                                      get_absolute_time()) >= 100000)
            {
                // Time to start a new read cycle
                if (slate->magnetorquers_running)
                {
                    // Turn off magnetorquers and wait for settle
                    stop_magnetorquer_pwm();
                    mag_torquer_off_time = get_absolute_time();
                    last_mag_read_start = get_absolute_time();
                    mag_state = MAG_WAITING_FOR_SETTLE;
                }
                else
                {
                    // Magnetorquers already off, read directly
                    rm3100_error_t result =
                        rm3100_get_reading(&slate->b_body, &slate->b_body_raw);
                    slate->magnetometer_data_valid = (result == RM3100_OK);
                    slate->b_body_read_time = get_absolute_time();
                    slate->bdot_data_has_updated = true;
                    last_mag_read_start = get_absolute_time();

                    if (result != RM3100_OK)
                    {
                        LOG_ERROR("[sensor] Error reading magnetometer");
                    }
                    // Update attitude filter with magnetometer measurement
                    if (result == RM3100_OK && slate->af_is_initialized)
                    {
                        LOG_DEBUG("[sensor] b_body = [%.3f, %.3f, %.3f]",
                                  slate->b_body.x, slate->b_body.y,
                                  slate->b_body.z);
                        attitude_filter_update(slate, 'M');
                    }
                    // Stay in IDLE
                }
            }
            break;

        case MAG_WAITING_FOR_SETTLE:
            // Check if settle time has elapsed
            if (absolute_time_diff_us(mag_torquer_off_time,
                                      get_absolute_time()) >=
                MAGNETOMETER_FIELD_SETTLE_TIME_MS * 1000) // Convert ms to us
            {
                mag_state = MAG_READING;
            }
            break;

        case MAG_READING:
        {
            // Read magnetometer
            rm3100_error_t result =
                rm3100_get_reading(&slate->b_body, &slate->b_body_raw);
            slate->magnetometer_data_valid = (result == RM3100_OK);
            slate->b_body_read_time = get_absolute_time();
            slate->bdot_data_has_updated = true;

            LOG_DEBUG("[sensor] b_body = [%.3f, %.3f, %.3f]", slate->b_body.x,
                      slate->b_body.y, slate->b_body.z);

            // Update attitude filter with magnetometer measurement
            if (result == RM3100_OK && slate->af_is_initialized)
            {
                attitude_filter_update(slate, 'M');
            }

            mag_state = MAG_RESTARTING_TORQUERS;
            break;
        }

        case MAG_RESTARTING_TORQUERS:
        {
            // Restart magnetorquers with last requested moment
            bool mag_result = do_magnetorquer_pwm(slate->magnetorquer_moment);
            if (!mag_result)
            {
                LOG_ERROR("[sensor] Error reactivating magnetorquers");
            }
            slate->magnetorquers_running = true;
            mag_state = MAG_IDLE;
            break;
        }
    }
}

sched_task_t magnetometer_task = {
    .name = "magnetometer",
    .dispatch_period_ms = 20, // 50 Hz dispatch for timing, 10 Hz mag data rate
                              // if magnetorquers are running
    .task_init = &magnetometer_task_init,
    .task_dispatch = &magnetometer_task_dispatch,
    .next_dispatch = 0};
