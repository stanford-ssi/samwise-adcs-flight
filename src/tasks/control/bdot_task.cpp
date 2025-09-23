/**
 * @author Niklas Vainio
 * @date 2025-06-02
 *
 * This task is responsible for running the B-dot detumbling algorithm
 *
 */

#include "bdot_task.h"
#include "gnc/control/bdot.h"
#include "gnc/utils/utils.h"
#include "macros.h"

void bdot_task_init(slate_t *slate)
{
    LOG_INFO("[bdot] Init Bdot task");
    // Reset flags
    slate->bdot_has_prev_data = false;
    slate->bdot_data_has_updated = false;
}

void bdot_task_dispatch(slate_t *slate)
{
    LOG_INFO("[bdot] Bdot task dispatching...");
    // If no new data flagged by sensor task, do nothing
    if (!(slate->bdot_data_has_updated && slate->magmeter_data_valid))
    {
        LOG_DEBUG("[bdot] No new data - returning!");
        return;
    }

    // If no previous data, bookmark and return
    if (!slate->bdot_has_prev_data)
    {
        LOG_DEBUG("[bdot] First cycle - returning!");
        slate->b_field_local_prev = slate->b_field_local;
        slate->b_field_read_time_prev = slate->b_field_read_time;

        slate->bdot_has_prev_data = true;
        slate->bdot_data_has_updated = false;
        return;
    }

    // We have data: compute control torques and dispatch to actuators
    float3 dB = slate->b_field_local - slate->b_field_local_prev;
    float dt = time_diff_seconds(slate->b_field_read_time_prev,
                                 slate->b_field_read_time);

    // float3 moments = bdot_compute_control_proportional(dB, dt);
    float3 moments = bdot_compute_control_bang_bang(dB, dt);

    LOG_DEBUG("[bdot] Requesting moments: x=%1.3f, y=%1.3f, z=%1.3f", moments.x,
              moments.y, moments.z);

    slate->magdrv_requested = moments;

    // Bookmark flags and return
    slate->bdot_data_has_updated = false;
}

sched_task_t bdot_task = {.name = "bdot",
                          .dispatch_period_ms = 100,
                          .task_init = &bdot_task_init,
                          .task_dispatch = &bdot_task_dispatch,

                          /* Set to an actual value on init */
                          .next_dispatch = 0};
