/**
 * @author  The ADCS Team :3
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
#include "macros.h"
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
    adcs_packet_t telemetry;

    // Watchdog
    bool watchdog_alive;
    bool watchdog_pin_high;
    absolute_time_t watchdog_pin_high_time;
    absolute_time_t watchdog_last_pet_time;

    // ========================================================================
    //          SENSOR DATA
    // ========================================================================

    // Magnetometer
    bool magnetometer_alive;
    bool magnetometer_data_valid;
    absolute_time_t b_body_read_time;
    float3 b_body_raw; // magnetic field in body frame (values in nT)
    float3 b_body;     // (unit vector)

    // GPS
    bool gps_alive;
    bool gps_data_valid;
    absolute_time_t gps_read_time; // time of last GPS read
    float gps_lat;                 // º (N+ S-)
    float gps_lon;                 // º (E+ W-)
    float gps_alt;                 // km
    float gps_time;                // utc = gps_time - 18 leap seconds

    // Sun sensors
    bool sun_sensor_alive[NUM_SUN_SENSORS]; // first 8 from RP2350B ADCs, last
                                            // 8 from photodiodes
    bool sun_sensor_data_valid[NUM_SUN_SENSORS];
    uint16_t
        sun_sensor_intensities[NUM_SUN_SENSORS]; // [0-3102] clipped to (2.5V
                                                 // / 3.3V) * 4095 due to
                                                 // differing reference voltages
    float sun_sensor_voltages[NUM_SUN_SENSORS];  // [V] voltage readings from
                                                 // sensors

    // Sun vector
    bool sun_vector_valid;       // true if sun vector is valid
    float3 sun_vector_body;      // (unit vector) in body frame
    float3 sun_vector_principal; // (unit vector) in principal axes frame

    // IMU
    float imu_alive;
    float imu_data_valid;
    float3 w_body_raw; // [rad/s] in body frame, raw reading
    float3 w_body;     // [rad/s] in body frame, low-pass filtered
    float w_mag;       // [rad/s] overall magnitude in body frame

    // Power monitor
    float adcs_power;         // [W] ADCS board power consumption
    float adcs_voltage;       // [V] ADCS board voltage
    float adcs_current;       // [A] ADCS board current
    bool power_monitor_alive; // true if ADM1176 power monitor is initialized

    // ========================================================================
    //          ACTUATOR REQUESTS
    // ========================================================================

    // Magnetorquer drivers
    bool magnetorquers_running; // true if magnetorquers are currently active
    float3 magnetorquer_moment; // magnetic moment [-1.0 to 1.0] in body frame

    // Reaction wheels
    bool
        reaction_wheels_running; // true if reaction wheels are currently active
    float w_reaction_wheels_requested[NUM_REACTION_WHEELS]; // [rad/s] in body
                                                            // frame
    float w_reaction_wheels[NUM_REACTION_WHEELS]; // [rad/s] in body frame, read
                                                  // from motor board feedback

    // ========================================================================
    //          GNC State
    // ========================================================================

    // General world state
    float3 UTC_date; // [year, month, day]
    float UTC_time;  // [seconds]
    float MJD;       // Modified Julian Date

    float3 sun_vector_eci; // (unit vector)

    // Magnetic field reference
    float3 b_eci_raw; // Magnetic field in ECI frame (values in nT)
    float3 b_ecef;    // Magnetic field in ECEF frame (unit vector)
    float3 b_eci;     // Magnetic field in ECI frame (unit vector)

    // Bdot
    float3 b_body_prev; // (unit vector)
    absolute_time_t b_body_read_time_prev;
    bool bdot_has_prev_data;
    bool bdot_data_has_updated;

    // Attitude filter
    bool af_is_initialized; // true if attitude filter has been
                            // initialized
    int af_init_count; // number of times attitude filter has been initialized
    absolute_time_t af_last_propagate_time; // time of last propagate call
    float P[6 * 6];                         // attitude covariance matrix (6x6)
                    // used in attitude filter, so no need for principal axes
    float P_log_frobenius; // log frobenius norm of attitude
                           // covariance
    quaternion
        q_eci_to_body; // scalar-last x,y,z,w in body frame. no inertia tensor
    float3 p_eci_to_body; // modified Rodrigues parameter from ECI to body frame
    float3 b_gyro_drift;  // gyro drift
    float3 tau_body;      // [Nm] total torque in body frame

    // Guidance 
    quaternion q_desired; // desired attitude quaternion in eci to principal axes frame
    float3 w_desired;     // [rad/s] desired angular velocity in principal axes frame
    
    // Attitude control
    float3 tau_control_principal; // [Nm] total torque in principal axes frame
    float3 tau_rw_principal; // [Nm] torque from reaction wheels in principal
                             // axes frame
    float3 tau_mt_principal; // [Nm] torque from magnetorquers in principal

    // Position
    float3 r_ecef;
    float3 r_eci;

// ========================================================================
//         TESTING  / EXPERIMENTAL FIELDS
// ========================================================================
#ifdef TEST
    float3 b_rpt; // Magnetic field in RTP frame (unit vector)
    float3 b_enu; // Magnetic field in ENU frame (unit vector)
#endif
} slate_t;