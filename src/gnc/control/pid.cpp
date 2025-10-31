/**
 * @author Lundeen Cahilly
 * @date 2025-10-24
 *
 * This file contains the PID control law for attitude control using quaternions
 * See page 10 of this paper
 * https://s3vi.ndc.nasa.gov/ssri-kb/static/resources/Attitude%20Determination%20&%20Control%20System%20Design%20and%20Implementation.pdf
 */

#include "pid.h"

#include "constants.h"
#include "gnc/utils/matrix_utils.h"
#include "gnc/utils/utils.h"
#include <cmath>

// TODO: tune gains!!!
constexpr float Kp = 0.1f;  // Proportional gain
constexpr float Ki = 0.01f; // Integral gain
constexpr float Kd = 0.05f; // Derivative gain

constexpr float INTEGRAL_ERROR_THRESHOLD =
    30.0f * DEG_TO_RAD; // [rad] - don't integrate when error is large
constexpr float INTEGRAL_ERROR_DECAY_RATE =
    0.95f; // fraction of integral error to decay per second (TODO: tune)
constexpr float3 MAX_TORQUE = {0.0f, 0.0f,
                               0.0f}; // TODO: @nablaxcroissant what is this lol

/**
 * Reset PID controller state (clears integral term)
 * Call this when starting a new maneuver or switching control modes
 *
 * @param slate Pointer to the slate structure
 */
void pid_init(slate_t *slate)
{
    slate->error_i = {0.0f, 0.0f, 0.0f};
}

/**
 * Compute the control torque using a PID controller based on quaternion error.
 *
 * @param slate Pointer to the slate structure containing current state
 * @return Control torque in body frame [Nm]
 */
void compute_control_torque_pid(slate_t *slate)
{
    // q_err represents the rotation from current to desired, expressed in body
    // frame (active)
    quaternion q_err = qmul(qconj(slate->q_eci_to_body), slate->q_desired);

    // Ensure shortest path
    if (q_err.w < 0.0f)
    {
        q_err = -q_err;
    }

    // Extract full axis-angle representation (works for ALL rotation angles)
    // error = angle * axis (rotation vector in body frame)
    float angle = qangle(q_err);
    float3 axis = qaxis(q_err);
    float3 error_p = angle * axis;

    // Compute derivative error
    float3 error_d =
        slate->w_desired - slate->w_body; // desired will almost always be zero

    // Compute integral error with anti-windup
    // Only integrate when error is small enough (conditional integration)
    float error_magnitude = length(error_p);
    if (error_magnitude < INTEGRAL_ERROR_THRESHOLD)
    {
        // Slide window integral error forward in time via decay
        slate->error_i = error_p + slate->error_i * INTEGRAL_ERROR_DECAY_RATE;
    }

    slate->tau_control_body = Kp * error_p + Ki * slate->error_i - Kd * error_d;
}

#ifdef TEST
// TODO: add tests!
#endif