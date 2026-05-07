#include "pico/time.h"

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

void vTaskBDot(void *) {
    for(;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_BDOT))
        TASK_LOOP_MS(100) // 10 Hz
 
        LOG_INFO("[bdot] Bdot task dispatching...");

        // If no new data flagged by sensor task, do nothing
        if (!(slate.bdot.bdot_data_has_updated_ 
                    && slate.magnetometer_data.magnetometer_data_valid))
        {
            LOG_DEBUG("[bdot] No new data - returning!");
            return;
        }

        // If no previous data, bookmark and return
        if (!slate.bdot.bdot_has_prev_data_)
        {
            LOG_DEBUG("[bdot] First cycle - returning!");
            slate.bdot.b_body_prev_ = slate.magnetometer_data.b_body;
            slate.bdot.b_body_read_time_prev_ = 
                slate.magnetometer_data.b_body_read_time;

            slate.bdot.bdot_has_prev_data_ = true;
            slate.bdot.bdot_data_has_updated_ = false;
            return;
        }

        // We have data - compute control moments
        float3 dB = slate.magnetometer_data.b_body - slate.bdot.b_body_prev_;
        float dt = time_diff_seconds(slate.bdot.b_body_read_time_prev_,
                                     slate.magnetometer_data.b_body_read_time);

        float3 moments = bdot_compute_control_bang_bang(
            dB, dt); // Could also use proportional if wanted

        LOG_DEBUG("[bdot] Requesting moments: x=%1.3f, y=%1.3f, z=%1.3f", moments.x,
                  moments.y, moments.z);

        slate.magtorq.magtorq_requested = moments;

        // Bookmark flags and return
        slate.bdot.bdot_data_has_updated_ = false;

    }

}

void vTaskActuators(void *) {
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_BDOT))
        TASK_LOOP_MS(100) // 10 Hz

        bool mag_result = do_magnetorquer_pwm(slate.magtorq.magtorq_duty_cycle);
        slate.magtorq.magnetorquers_running = mag_result;

        if (!mag_result)
        {
            LOG_ERROR("[actuators] Magnetorquer PWM error");
        }

    } // end for(;;)
}
