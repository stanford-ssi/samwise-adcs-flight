/**
 * @author Lundeen Cahilly
 * @date 2025-11-08
 *
 * This task is responsible for allocating the magnetorquer dipole moments
 * given the requested control moment
 */


#include "magtorq_allocation.h"
#include "constants.h"

static float magnetorquer_power(slate_t* slate, float3 duty_cycles) {
    // P = I^2R
    float voltage = V_BATT_MAX; // [V] assume worst case - max voltage from batteries
    if (slate->power_monitor_alive) {
        voltage = slate->adcs_voltage;
    }
    float3 current = duty_cycles * voltage / MAGTORQ_RESISTANCE;
    return dot(current * current, MAGTORQ_RESISTANCE);
}

/**
 * @brief Allocate magnetorquer dipoles given requested control moment using projection
 * of requested moment onto magnetorquer axes, clamped by relative strengths
 *
 * @param slate Pointer to the current satellite slate
 */
void allocate_magnetorquers(slate_t *slate) {
    float3 requested_moment = slate->magtorq_requested;

    // Project requested moment onto magnetorquer axes
    float3 allocated_dipole = requested_moment / MAGTORQ_MOMENTS;

    // Clamp individual duty cycles to [-1, 1]
    float3 duty_cycle = allocated_dipole;
    duty_cycle.x = fminf(1.0f, fmaxf(-1.0f, duty_cycle.x));
    duty_cycle.y = fminf(1.0f, fmaxf(-1.0f, duty_cycle.y));
    duty_cycle.z = fminf(1.0f, fmaxf(-1.0f, duty_cycle.z));

    // Adjust duty cycle to not exceed max power
    float power = magnetorquer_power(slate, duty_cycle);
    if (power > MAGTORQ_MAX_POWER) {
        float scale = sqrt(MAGTORQ_MAX_POWER / power);
        duty_cycle = duty_cycle * scale;
    }

    // Update allocated dipole based on final duty cycle
    allocated_dipole = duty_cycle;

    LOG_DEBUG("[magtorq_allocation] Requested moment: (%f, %f, %f) Am^2\n",
              requested_moment.x, requested_moment.y, requested_moment.z);
    LOG_DEBUG("[magtorq_allocation] Allocated dipole: (%f, %f, %f) Am^2\n",
              allocated_dipole.x * MAGTORQ_MOMENTS.x,
              allocated_dipole.y * MAGTORQ_MOMENTS.y,
              allocated_dipole.z * MAGTORQ_MOMENTS.z);
    LOG_DEBUG("[magtorq_allocation] Duty cycle: (%f, %f, %f)\n",
              duty_cycle.x, duty_cycle.y, duty_cycle.z);
    LOG_DEBUG("[magtorq_allocation] Power consumption: %f W\n",
              magnetorquer_power(slate, duty_cycle));

    slate->magtorq_duty_cycle = duty_cycle;
    slate->magtorq_moment = allocated_dipole * MAGTORQ_MOMENTS;
}

#ifdef TEST
/**
 * @brief Test that allocated moment matches requested direction
 */
void test_magtorq_allocation(slate_t *slate) {
    slate->power_monitor_alive = false;

    LOG_INFO("[test_magtorq_allocation] Testing moment direction preservation\n");

    // Test with a few different requested moments
    float3 test_cases[] = {
        {0.01f, 0.02f, 0.005f},
        {0.03f, 0.03f, 0.015f},
        {1.0f, 1.0f, 1.0f}  // Max moments
    };

    for (int i = 0; i < 3; i++) {
        slate->magtorq_requested = test_cases[i];
        allocate_magnetorquers(slate);

        // Normalize both vectors to compare direction
        float req_mag = sqrt(dot(slate->magtorq_requested, slate->magtorq_requested));
        float out_mag = sqrt(dot(slate->magtorq_moment, slate->magtorq_moment));

        float3 req_dir = slate->magtorq_requested / req_mag;
        float3 out_dir = slate->magtorq_moment / out_mag;

        // Check if directions match (dot product should be ~1.0)
        float alignment = dot(req_dir, out_dir);

        LOG_INFO("Test %d: Requested (%f, %f, %f) -> Allocated (%f, %f, %f)",
                 i + 1,
                 slate->magtorq_requested.x, slate->magtorq_requested.y, slate->magtorq_requested.z,
                 slate->magtorq_moment.x, slate->magtorq_moment.y, slate->magtorq_moment.z);
        LOG_INFO("        Direction alignment: %f %s\n",
                 alignment,
                 (alignment > 0.99f) ? "PASS" : "FAIL");
    }
}
#endif