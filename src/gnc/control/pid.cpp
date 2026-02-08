/**
 * @author Lundeen Cahilly
 * @date 2025-10-24
 *
 * This file contains the PID control law for attitude control using quaternions
 * See page 10 of this paper
 * https://s3vi.ndc.nasa.gov/ssri-kb/static/resources/Attitude%20Determination%20&%20Control%20System%20Design%20and%20Implementation.pdf
 */

#include "pid.h"

#include "gnc/utils/matrix_utils.h"
#include "gnc/utils/utils.h"
#include "macros.h"
#include "params.h"
#include <cmath>

// TODO: tune gains!!!
constexpr float Kp = 0.1f;  // Proportional gain
constexpr float Ki = 0.01f; // Integral gain
constexpr float Kd = 0.05f; // Derivative gain

constexpr float INTEGRAL_ERROR_THRESHOLD =
    30.0f * DEG_TO_RAD; // [rad] - don't integrate when error is large
constexpr float INTEGRAL_ERROR_DECAY_RATE =
    0.995f; // fraction of integral error to decay per second (TODO: tune)
constexpr float3 MAX_TORQUE = {
    1.0e-3f, 1.0e-3f, 1.0e-3f}; // 1 mNm per axis TODO: configure for our
                                // tetrahehdral mount, should be in constants.h

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
 * @param dt Time step in seconds
 * @return Control torque in body frame [Nm]
 */
void compute_control_torque_pid(slate_t *slate, float dt)
{
    // Rotation from current to desired (body frame)
    quaternion q_err = qmul(qconj(slate->q_eci_to_body), slate->q_desired);

    // Ensure shortest path
    if (q_err.w < 0.0f)
    {
        q_err = -q_err;
    }

    // Proportional error (vector part)
    float3 error_p = {q_err.x, q_err.y, q_err.z};

    // Derivative error
    float3 error_d = slate->w_desired - slate->w_body;

    // Integral error with anti-windup
    float error_magnitude = length(error_p);
    if (error_magnitude < INTEGRAL_ERROR_THRESHOLD)
    {
        slate->error_i =
            (slate->error_i + error_p * dt) * INTEGRAL_ERROR_DECAY_RATE;
    }
    else
    {
        slate->error_i = slate->error_i * INTEGRAL_ERROR_DECAY_RATE;
    }

    // PID Control Law
    slate->tau_control_body = Kp * error_p + Ki * slate->error_i + Kd * error_d;

    // Saturation
    for (int i = 0; i < 3; i++)
    {
        if (slate->tau_control_body[i] > MAX_TORQUE[i])
        {
            slate->tau_control_body[i] = MAX_TORQUE[i];
        }
        else if (slate->tau_control_body[i] < -MAX_TORQUE[i])
        {
            slate->tau_control_body[i] = -MAX_TORQUE[i];
        }
    }
}

#ifdef TEST
#include <cassert>

void test_pid_proportional(slate_t *slate)
{
    LOG_DEBUG("Testing Proportional Control...");
    pid_init(slate);

    slate->q_desired = {0, 0, 0, 1};
    slate->q_eci_to_body = {0.70710678f, 0, 0, 0.70710678f}; // 90 deg about X
    slate->w_desired = {0, 0, 0};
    slate->w_body = {0, 0, 0};

    compute_control_torque_pid(slate, 0.1f);

    LOG_DEBUG("tau_control_body: %f %f %f", slate->tau_control_body.x,
              slate->tau_control_body.y, slate->tau_control_body.z);

    // Check saturation (-1e-3)
    assert(std::abs(slate->tau_control_body.x + 0.001f) < 1e-5f);
    assert(std::abs(slate->tau_control_body.y) < 1e-5f);
    assert(std::abs(slate->tau_control_body.z) < 1e-5f);

    LOG_DEBUG("PASS");
}

void test_pid_derivative(slate_t *slate)
{
    LOG_DEBUG("Testing Derivative Control...");
    pid_init(slate);

    slate->q_desired = {0, 0, 0, 1};
    slate->q_eci_to_body = {0, 0, 0, 1};
    slate->w_desired = {0, 0, 0};
    slate->w_body = {1.0f, 0, 0}; // Spinning X

    compute_control_torque_pid(slate, 0.1f);

    LOG_DEBUG("tau_control_body: %f %f %f", slate->tau_control_body.x,
              slate->tau_control_body.y, slate->tau_control_body.z);

    // Check saturation (-1e-3)
    assert(std::abs(slate->tau_control_body.x + 0.001f) < 1e-5f);

    LOG_DEBUG("PASS");
}

void test_pid_integral(slate_t *slate)
{
    LOG_DEBUG("Testing Integral Control...");
    pid_init(slate);

    // Small error (1 deg about X)
    slate->q_desired = {0, 0, 0, 1};
    slate->q_eci_to_body = {0.008726f, 0, 0, 0.99996f};
    slate->w_desired = {0, 0, 0};
    slate->w_body = {0, 0, 0};

    float dt = 1.0f;

    compute_control_torque_pid(slate, dt);
    float first_integral_x = slate->error_i.x;
    assert(first_integral_x < 0);

    LOG_DEBUG("tau_control_body: %f %f %f", slate->tau_control_body.x,
              slate->tau_control_body.y, slate->tau_control_body.z);

    compute_control_torque_pid(slate, dt);
    assert(slate->error_i.x < first_integral_x); // Should accumulate

    LOG_DEBUG("PASS");
}
#endif