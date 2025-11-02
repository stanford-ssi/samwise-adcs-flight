/**
 * @author  Niklas Vainio
 * @date    2025-05-29
 *
 * This task is responsible for sending telemetry to the picubed. It runs in all
 * states.
 */

#include "telemetry_task.h"
#include <cstring>
#include "drivers/picubed/picubed.h"
#include "macros.h"

/**
 * @brief Get current state as a single character
 *
 * @param slate Pointer to the current satellite slate
 * @return Character representing the current state
 */
static char get_state_char(slate_t *slate)
{
    const char *name = slate->current_state->name;
    // do same as below but with names use map if necessary
    if (strcmp(name, "init") == 0)
        return 'I';
    if (strcmp(name, "detumble") == 0)
        return 'D';
    if (strcmp(name, "slewing") == 0)
        return 'S';
    if (strcmp(name, "cool_down") == 0)
        return 'C';
    if (strcmp(name, "test") == 0)
        return 'T';
    else
        return '?';
}

/**
 * @brief Populate telemetry data in the slate with real sensor values.
 *
 * @param slate Pointer to the current satellite slate
 */
static void populate_telemetry(slate_t *slate)
{
    slate->telemetry = {

        .state = get_state_char(slate),
        .boot_count = slate->boot_count,

        // Quaternion (only x,y,z components)
        .q_eci_to_body_x = slate->q_eci_to_body.x,
        .q_eci_to_body_y = slate->q_eci_to_body.y,
        .q_eci_to_body_z = slate->q_eci_to_body.z,
        .q_eci_to_body_w = slate->q_eci_to_body.w,

        // Angular velocity
        .w_body_x = slate->w_body.x,
        .w_body_y = slate->w_body.y,
        .w_body_z = slate->w_body.z,

        // Power
        .adcs_power = slate->adcs_power,

        // Sun vector
        .sun_vector_body_x = slate->sun_vector_body.x,
        .sun_vector_body_y = slate->sun_vector_body.y,
        .sun_vector_body_z = slate->sun_vector_body.z,

        // Magnetic field
        .b_body_raw_x = slate->b_body_raw.x,
        .b_body_raw_y = slate->b_body_raw.y,
        .b_body_raw_z = slate->b_body_raw.z,

        // Magnetorquer moment
        .magnetorquer_moment_x = slate->magnetorquer_moment.x,
        .magnetorquer_moment_y = slate->magnetorquer_moment.y,
        .magnetorquer_moment_z = slate->magnetorquer_moment.z,

        // Reaction wheels
        .w_reaction_wheels_0 = slate->w_reaction_wheels[0],
        .w_reaction_wheels_1 = slate->w_reaction_wheels[1],
        .w_reaction_wheels_2 = slate->w_reaction_wheels[2],
        .w_reaction_wheels_3 = slate->w_reaction_wheels[3],

        // Validity flags
        .magnetometer_data_valid = slate->magnetometer_data_valid,
        .gps_data_valid = slate->gps_data_valid,
        .imu_data_valid = slate->imu_data_valid,
        .sun_vector_valid = slate->sun_vector_valid,

        // Covariance
        .P_log_frobenius = slate->P_log_frobenius,

        // GPS
        .lat = slate->gps_lat,
        .lon = slate->gps_lon,
        .alt = slate->gps_alt,

        // Time
        .mjd = slate->MJD};

    // Copy sun sensor data validity array (16 bools)
    for (int i = 0; i < NUM_SUN_SENSORS; i++)
    {
        slate->telemetry.sun_sensor_data_valid[i] =
            slate->sun_sensor_data_valid[i];
    }
}

/**
 * @brief Initialize telemetry task. Initializes picubed UART.
 *
 * @param slate Pointer to the current satellite slate
 */
void telemetry_task_init(slate_t *slate)
{
    LOG_INFO("[telem] Initializing picubed UART...");

    picubed_uart_init();

    LOG_INFO("[telem] Picubed UART successfully initialized!");
}

/**
 * @brief Dispatch telemetry task. Populates telemetry data and handles
 * picubed commands.
 *
 * @param slate Pointer to the current satellite slate
 */
void telemetry_task_dispatch(slate_t *slate)
{
    LOG_INFO("[telem] Telemetry task dispatching...");
    populate_telemetry(slate);
    picubed_uart_handle_commands(slate);
}

sched_task_t telemetry_task = {.name = "telem",
                               .dispatch_period_ms =
                                   100, // TODO: determine appropriate rate
                               .task_init = &telemetry_task_init,
                               .task_dispatch = &telemetry_task_dispatch,

                               /* Set to an actual value on init */
                               .next_dispatch = 0};
