/**
 * @author Lundeen Cahilly
 * @date 2025-10-18
 *
 * Sun sensor task for reading photodiode ADCs and computing sun vector.
 * Runs at 20 Hz for fast attitude observations.
 */

#include "sun_sensor_task.h"
#include "constants.h"
#include "macros.h"

#include "drivers/sun_sensors/ads7830.h"
#include "drivers/sun_sensors/rp2350b_adc.h"
#include "gnc/estimation/attitude_filter.h"
#include "gnc/estimation/sun_sensor_to_vector.h"

/**
 * @brief Initialize sun sensor task.
 *
 * @param slate Pointer to the current satellite slate
 */
void sun_sensor_task_init(slate_t *slate)
{
    LOG_INFO("[sensor] Initializing sun sensors rp2350b adc...");
    bool rp2350b_adc_result = rp2350b_adc_init();
    if (!rp2350b_adc_result)
    {
        LOG_ERROR("[sensor] Error initializing rp2350b_adc - deactivating!");
    }

    LOG_INFO("[sensor] Initializing sun sensors ads7830...");
    bool ads7830_result = ads7830_init();
    if (!ads7830_result)
    {
        LOG_ERROR("[sensor] Error initializing ads7830 - deactivating!");
    }

    // Write alive status for each sun sensor
    for (uint8_t i = 0; i < NUM_SUN_SENSORS; i++)
    {
        if (i < NUM_SUN_SENSORS / 2)
        {
            // First 8 sensors are from rp2350b adc
            slate->sun_sensor_alive[i] = rp2350b_adc_result;
        }
        else
        {
            // Last 8 sensors are from ads7830
            slate->sun_sensor_alive[i] = ads7830_result;
        }
        slate->sun_sensor_data_valid[i] = false;
    }

    slate->sun_vector_valid = false;

    LOG_INFO("[sensor] Sun sensors alive status:");
    for (uint8_t i = 0; i < NUM_SUN_SENSORS; i++)
    {
        LOG_INFO("  Sensor %2d: %s", i,
                 slate->sun_sensor_alive[i] ? "true" : "false");
    }
}

/**
 * @brief Dispatch sun sensor task. Reads ADCs and computes sun vector.
 *
 * @param slate Pointer to the current satellite slate
 */
void sun_sensor_task_dispatch(slate_t *slate)
{
    // --- Read rp2350b ADC (sensors 0-7) --- //
    bool rp2350b_adc_alive = slate->sun_sensor_alive[0];
    if (rp2350b_adc_alive)
    {
        uint16_t intensities[8];
        float voltages[8];
        bool result = rp2350b_adc_read_all_channels(intensities);
        bool voltage_result = rp2350b_adc_read_all_voltages(voltages);

        for (int i = 0; i < 8; i++)
        {
            if (result)
            {
                // Clip to 2.5V max value for consistency with ads7830
                uint16_t normalized_intensity =
                    min(intensities[i], SUN_SENSOR_CLIP_VALUE);
                slate->sun_sensor_intensities[i] = normalized_intensity;
                slate->sun_sensor_data_valid[i] = true;
            }
            if (voltage_result)
            {
                slate->sun_sensor_voltages[i] = voltages[i];
            }
        }
    }
    else
    {
        LOG_DEBUG(
            "[sensor] Skipping rp2350b_adc due to invalid initialization!");
    }

    // --- Read ads7830 ADC (sensors 8-15) --- //
    bool ads7830_alive = slate->sun_sensor_alive[8];
    if (ads7830_alive)
    {
        uint8_t intensities[8];
        float voltages[8];
        bool result = ads7830_read_all_channels(intensities);
        bool voltage_result = ads7830_read_all_voltages(voltages);

        for (int i = 0; i < 8; i++)
        {
            if (result)
            {
                // Scale 8-bit to 12-bit range for consistency
                uint16_t normalized_intensity =
                    static_cast<uint16_t>(intensities[i]) *
                    SUN_SENSOR_CLIP_VALUE / MAX_VALUE_ADS7830;
                slate->sun_sensor_intensities[i + 8] = normalized_intensity;
                slate->sun_sensor_data_valid[i + 8] = true;
            }
            if (voltage_result)
            {
                slate->sun_sensor_voltages[i + 8] = voltages[i];
            }
        }
    }
    else
    {
        LOG_DEBUG("[sensor] Skipping ads7830 due to invalid initialization!");
    }

    // --- Compute sun vector in body frame --- //
    sun_sensors_to_vector(slate);

    if (slate->sun_vector_valid)
    {
        LOG_DEBUG("[sensor] sun_vector_body = [%.3f, %.3f, %.3f]",
                  slate->sun_vector_body.x, slate->sun_vector_body.y,
                  slate->sun_vector_body.z);

        if (slate->af_is_initialized)
        {
            attitude_filter_update(slate, 'S'); // Update with sun vector
        }
    }
}

sched_task_t sun_sensor_task = {.name = "sun_sensors",
                                .dispatch_period_ms = 50, // 20 Hz
                                .task_init = &sun_sensor_task_init,
                                .task_dispatch = &sun_sensor_task_dispatch,
                                .next_dispatch = 0};
