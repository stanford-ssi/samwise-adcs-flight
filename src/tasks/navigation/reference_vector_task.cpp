/**
 * @author Lundeen Cahilly
 * @date 2025-10-18
 *
 * Reference vector task. Computes ECI reference vectors (sun_vector_eci, b_eci)
 * based on current orbit state (r_eci) and time. Runs at 50 Hz (same as IMU)
 * to keep pace with orbit filter propagation.
 */

#include "reference_vector_task.h"
#include "constants.h"
#include "macros.h"

#include "gnc/world/b_field.h"
#include "gnc/world/sun_vector.h"

/**
 * @brief Initialize reference vector task.
 *
 * @param slate Pointer to the current satellite slate
 */
void reference_vector_task_init(slate_t *slate)
{
    LOG_INFO("[world] Initializing reference vector task...");
    // No hardware initialization needed - just computation
}

/**
 * @brief Dispatch reference vector task.
 *
 * Computes reference vectors (b_eci, sun_vector_eci) based on GPS
 * position/time. Only updates when GPS data is valid.
 *
 * @param slate Pointer to the current satellite slate
 */
void reference_vector_task_dispatch(slate_t *slate)
{
    // Only compute reference vectors if GPS data is valid
    if (!slate->gps_data_valid)
    {
        LOG_DEBUG(
            "[world] Skipping reference vector update - GPS data invalid");
        return;
    }

    // Compute sun vector in ECI frame
    compute_sun_vector_eci(slate);

    // Compute magnetic field vector in ECI frame
    compute_B(slate);

    LOG_DEBUG("[world] sun_vector_eci = [%.6f, %.6f, %.6f]",
              slate->sun_vector_eci.x, slate->sun_vector_eci.y,
              slate->sun_vector_eci.z);
    LOG_DEBUG("[world] b_eci = [%.3f, %.3f, %.3f]", slate->b_eci.x,
              slate->b_eci.y, slate->b_eci.z);
}

sched_task_t reference_vector_task = {
    .name = "ref_vectors",
    .dispatch_period_ms = 20, // 50 Hz (same as IMU/orbit filter propagation)
    .task_init = &reference_vector_task_init,
    .task_dispatch = &reference_vector_task_dispatch,
    .next_dispatch = 0};
