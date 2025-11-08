/**
 * @author Lundeen Cahilly
 * @date 2025-11-08
 *
 * This task is responsible for allocating the magnetorquer dipole moments
 * given the requested control moment
 */


#include "magtorq_allocation.h"
#include "constants.h"

/**
 * @brief Allocate magnetorquer dipoles given requested control moment using projection
 * of requested moment onto magnetorquer axes, clamped by relative strengths
 *
 * @param slate Pointer to the current satellite slate
 */
void allocate_magnetorquers(slate_t *slate) {
    // Simple allocation: for each axis, allocate dipole moment proportional to
    // requested moment, clamped to max duty cycle

    float3 requested_moment = slate->magnetorquer_moment;
    float3 max_duty = slate->magnetorquer_max_duty;

    float3 allocated_dipole;
    for (int i = 0; i < 3; i++) {
        // Assuming linear relationship between moment and dipole for simplicity
        float dipole = requested_moment[i] / MAGNETORQUER_MOMENT_CONSTANT;

        // Clamp to max duty cycle
        if (dipole > max_duty[i]) {
            dipole = max_duty[i];
        } else if (dipole < -max_duty[i]) {
            dipole = -max_duty[i];
        }

        allocated_dipole[i] = dipole;
    }

    // Update slate with allocated dipole moments
    slate->magnetorquer_moment = allocated_dipole;
}