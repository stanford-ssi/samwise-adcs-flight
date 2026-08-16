#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "pico/time.h"

#include <math.h>

#include "macros.h"

#include "linalg.h"
using namespace linalg;
using namespace linalg::aliases;

#include "drivers/magnetorquers/magnetorquers.h"

#include "apps/adcs_new/tasks/tasks.h"
#include "apps/adcs_new/params.h"
#include "apps/adcs_new/slate.h"

#include "gnc/utils/utils.h"
#include "gnc/control/bdot.h"

extern slate_t slate;

/*
 * Electrical power the magnetorquers would draw at the given duty cycles.
 *
 * P = sum over axes of (duty * V)^2 / R. Uses the measured bus voltage when we
 * have it and the worst case otherwise, so an unknown bus never looks cheap.
 */
static float magnetorquer_power(float3 duty_cycle)
{
    float voltage = V_BATT_MAX;
    if (slate.power_monitor.power_monitor_alive)
    {
        voltage = slate.power_monitor.voltage;
    }
    const float3 current = duty_cycle * voltage / R_MAGTORQ;
    return dot(current * current, R_MAGTORQ);
}

/*
 * Turn the requested control moment into per-axis PWM duty cycles.
 *
 * bdot_compute_control_bang_bang() already returns values in [-1, 1], i.e. duty
 * cycle space rather than A m^2, which is also what do_magnetorquer_pwm()
 * expects. So this only has to enforce the limits:
 *
 *   1. clamp uniformly into [-1, 1], preserving direction
 *   2. scale uniformly to stay inside MAGTORQ_MAX_POWER
 *
 * NOTE: duplicates the intent of gnc/actuators/magtorq_allocation.cpp, which
 * cannot be reused here - it is written against adcs_app's slate_t and is not
 * even compiled into this target.
 */
static void allocate_magnetorquers()
{
    float3 duty_cycle = slate.magtorq.magtorq_requested;

    // Scale the whole vector by the worst offender so the commanded direction
    // survives clamping.
    const float max_abs = fmaxf(fmaxf(fabsf(duty_cycle.x), fabsf(duty_cycle.y)),
                                fabsf(duty_cycle.z));
    if (max_abs > 1.0f)
    {
        duty_cycle = duty_cycle / max_abs;
    }

    const float power = magnetorquer_power(duty_cycle);
    if (power > MAGTORQ_MAX_POWER)
    {
        duty_cycle = duty_cycle * sqrtf(MAGTORQ_MAX_POWER / power);
    }

    slate.magtorq.magtorq_duty_cycle = duty_cycle;
    slate.magtorq.magtorq_moment = duty_cycle * MU_MAGTORQ;

    LOG_DEBUG("[actuators] duty = [%.3f, %.3f, %.3f], P = %.3f W",
              duty_cycle.x, duty_cycle.y, duty_cycle.z,
              magnetorquer_power(duty_cycle));
}

/*
 * Whether it is safe to energise the magnetorquers right now.
 *
 * Fails closed on every unknown: no monitor, no recent reading, or a bus below
 * the disarm threshold all mean no torquing. vTaskWatchdog owns magtorq_armed
 * and applies the hysteresis; this only re-checks that its verdict is current.
 */
static bool magtorq_power_is_safe()
{
    if (slate.state_machine.current_state != STATE_DETUMBLE)
    {
        LOG_DEBUG("[actuators] Inhibited: not in commanded detumble mode");
        return false;
    }
    if (!slate.power_monitor.power_monitor_alive)
    {
        LOG_DEBUG("[actuators] Inhibited: power monitor not alive");
        return false;
    }
    if (!slate.magtorq_armed)
    {
        LOG_DEBUG("[actuators] Inhibited: not armed");
        return false;
    }
    if (absolute_time_diff_us(slate.power_read_time, get_absolute_time()) >
        (int64_t)POWER_DATA_EXPIRATION_MS * 1000)
    {
        LOG_DEBUG("[actuators] Inhibited: power telemetry stale");
        return false;
    }
    return true;
}

void vTaskBDot(void *) {
    for(;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_BDOT))
        TASK_LOOP_MS(100) // 10 Hz

        LOG_INFO("[bdot] Bdot task dispatching...");

        // NOTE: every early exit below must `continue`, never `return`.
        // Returning from a FreeRTOS task body lands in prvTaskExitError, whose
        // configASSERT is `if (!(x)) for(;;)` - the task would spin forever at
        // priority 2 and B-dot would never actuate again.

        // If no new data flagged by the sensor task, do nothing
        if (!(slate.bdot.bdot_data_has_updated_
                    && slate.magnetometer_data.magnetometer_data_valid))
        {
            LOG_DEBUG("[bdot] No new data - skipping cycle");
            continue;
        }

        // If no previous data, bookmark and wait for the next reading
        if (!slate.bdot.bdot_has_prev_data_)
        {
            LOG_DEBUG("[bdot] First cycle - seeding previous reading");
            slate.bdot.b_body_prev_ = slate.magnetometer_data.b_body;
            slate.bdot.b_body_read_time_prev_ =
                slate.magnetometer_data.b_body_read_time;

            slate.bdot.bdot_has_prev_data_ = true;
            slate.bdot.bdot_data_has_updated_ = false;
            continue;
        }

        // We have data - compute control moments
        const float3 dB =
            slate.magnetometer_data.b_body - slate.bdot.b_body_prev_;
        const float dt = time_diff_seconds(slate.bdot.b_body_read_time_prev_,
                                     slate.magnetometer_data.b_body_read_time);

        if (dt <= 0.0f)
        {
            LOG_DEBUG("[bdot] Non-positive dt - skipping cycle");
            slate.bdot.bdot_data_has_updated_ = false;
            continue;
        }

        slate.magtorq.magtorq_requested = bdot_compute_control_bang_bang(
            dB, dt); // Could also use proportional if wanted

        // Turn the request into duty cycles. Nothing did this before, so
        // vTaskActuators was always commanding a duty cycle of zero.
        allocate_magnetorquers();

        LOG_DEBUG("[bdot] Requesting moments: x=%1.3f, y=%1.3f, z=%1.3f",
                  slate.magtorq.magtorq_requested.x,
                  slate.magtorq.magtorq_requested.y,
                  slate.magtorq.magtorq_requested.z);

        // Bookmark for the next cycle
        slate.bdot.b_body_prev_ = slate.magnetometer_data.b_body;
        slate.bdot.b_body_read_time_prev_ =
            slate.magnetometer_data.b_body_read_time;
        slate.bdot.bdot_data_has_updated_ = false;
    }
}

void vTaskActuators(void *) {
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_ACTUATORS))

        if (!magtorq_power_is_safe())
        {
            // Make sure the rods are actually off, not merely uncommanded.
            stop_magnetorquer_pwm();
            slate.magtorq.magnetorquers_running = false;
            vTaskDelay(pdMS_TO_TICKS(MAGTORQ_ON_TIME_MS +
                                     MAGTORQ_OFF_TIME_MS));
            continue;
        }

        // The rods swamp the magnetometer, so hold mag_mutex across the pulse
        // and the field decay, then release it for the whole off window. That
        // release is what gives vTaskMagnetometer a clean read; holding through
        // the off time starved it of roughly half its samples.
        if (xSemaphoreTake(slate.mag_mutex,
                           pdMS_TO_TICKS(MAG_MUTEX_TIMEOUT_MS)) != pdTRUE)
        {
            LOG_ERROR("[actuators] Timed out waiting for magnetometer mutex");
            continue;
        }

        // The state can change while this task waits for the magnetometer. Do
        // not begin a new pulse after DETUMBLE has been exited.
        if (!magtorq_power_is_safe())
        {
            stop_magnetorquer_pwm();
            slate.magtorq.magnetorquers_running = false;
            xSemaphoreGive(slate.mag_mutex);
            continue;
        }

        const bool mag_result =
            do_magnetorquer_pwm(slate.magtorq.magtorq_duty_cycle);
        slate.magtorq.magnetorquers_running = mag_result;
        if (!mag_result)
        {
            LOG_ERROR("[actuators] Magnetorquer PWM error");
        }
        vTaskDelay(pdMS_TO_TICKS(MAGTORQ_ON_TIME_MS));

        // Turn off and let the field decay before anyone reads the
        // magnetometer.
        stop_magnetorquer_pwm();
        slate.magtorq.magnetorquers_running = false;
        vTaskDelay(pdMS_TO_TICKS(MAGNETOMETER_FIELD_SETTLE_TIME_MS));

        xSemaphoreGive(slate.mag_mutex);

        // Magnetometer window: coils off, mutex free.
        vTaskDelay(pdMS_TO_TICKS(MAGTORQ_OFF_TIME_MS));
    } // end for(;;)
}
