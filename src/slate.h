/**
 * @author  The ADCS Team :)
 * @date    2024-02-08
 *
 * This file defines the slate struct, a static struct which stores all data on
 * the satellite. At init time, a single instance of this struct gets statically
 * allocated, and it is referenced by all tasks and functions.
 *
 * Look up "blackboard pattern" for more info.
 */

#pragma once

#include "adcs_packet.h"
#include "constants.h"
#include "linalg.h"
#include "pico/types.h"
#include "scheduler/state_machine_types.h"

using namespace linalg::aliases;
using namespace linalg;

typedef struct samwise_adcs_slate
{
    // ========================================================================
    //          GENERAL STATE
    // ========================================================================

    // State machine
    sched_state_t *current_state;
    absolute_time_t entered_current_state_time;
    uint32_t time_in_current_state_ms;

    // Telemetry
    adcs_packet_t telem;

    // Watchdog
    bool watchdog_initialized;
    bool pin_high;
    absolute_time_t pin_high_time;
    absolute_time_t last_feed_time;

    // ========================================================================
    //          SENSOR DATA
    // ========================================================================

    // Magmeter
    float3 b_field_local; // (unit vector)
    absolute_time_t b_field_read_time;
    bool magmeter_data_valid;
    bool magmeter_alive;

    // GPS
    float gps_lat; // º (N+ S-)
    float gps_lon; // º (E+ W-)
    float gps_alt; // km
    float gps_time;
    bool gps_data_valid;
    bool gps_alive;

    // Sun sensors
    uint16_t sun_sensors_intensities
        [NUM_SUN_SENSORS]; // [0-3102] clipped to (2.5V / 3.3V) * 4095 due to
                           // differing reference voltages
    float sun_sensors_voltages[NUM_SUN_SENSORS]; // [V] voltage readings from
                                                 // sensors
    bool sun_pyramids_data_valid; // include bc ADC chip could fail
    bool sun_pyramids_alive;
    bool photodiodes_yz_data_valid;
    bool photodiodes_yz_alive;

    // Sun vector
    bool sun_vector_valid;       // true if sun vector is valid
    float3 sun_vector_body;      // (unit vector) in body frame
    float3 sun_vector_principal; // (unit vector) in principal axes frame

    // IMU
    float3 w_body_raw;      // [rad/s] in body frame
    float3 w_body_filtered; // [rad/s] in body frame
    float w_mag;            // [rad/s] overall magnitude in body frame
    float imu_data_valid;
    float imu_alive;

    // Power monitoring
    float adcs_power;   // [W] ADCS board power consumption
    float adcs_voltage; // [V] ADCS board voltage
    float adcs_current; // [A] ADCS board current
    bool adm1176_alive; // true if ADM1176 is initialized

    // ========================================================================
    //          ACTUATOR REQUESTS
    // ========================================================================
    // Magnetorquer drivers
    float3 magdrv_requested;    // [-1.0 to 1.0] in body frame
    bool magnetorquers_running; // true if magnetorquers are currently active

    // Reaction wheels
    float reaction_wheels_w_requested[NUM_REACTION_WHEELS]; // [rad/s]

    // ========================================================================
    //          GNC State
    // ========================================================================
    // General world state
    float3 UTC_date; // [year, month, day]
    float UTC_time;  // [seconds]
    float MJD;       // Modified Julian Date

    // Simulation timing (only used in SIMULATION mode)
    float sim_t0_mjd;              // MJD of first simulator packet
    absolute_time_t sim_t0_system; // System time when first packet received
    bool sim_time_initialized;     // true after first packet

    float3 sun_vector_eci; // (unit vector)

    float3 B_est_rpt;  // R, phi, theta frame (unit vector) [Up, East, North]
    float3 B_est_enu;  // East-North-Up frame (unit vector)
    float3 B_est_ecef; // ECEF frame (unit vector)

    // Bdot
    float3 b_field_local_prev; // (unit vector)
    absolute_time_t b_field_read_time_prev;
    bool bdot_has_prev_data;
    bool bdot_data_has_updated;

    // Attitude propagator
    quaternion q_eci_to_principal; // principal axes frame
    float3 w_principal;            // [rad s^-1] in principal axes frame
    float3 tau_principal;          // [Nm] total torque in principal axes frame

    float attitude_covar[7 * 7]; // attitude covariance matrix

    // Attitude control
    float3 control_torque;
    float3 reaction_wheel_speeds;

    float3 r_ecef;
    float3 r_eci;

} slate_t;
