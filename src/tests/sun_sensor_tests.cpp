/**
 * @author Chen Li and Lundeen Cahilly
 * @date 2025-08-24
 *
 * Tests for sun sensor to vector conversion
 */

#include "sun_sensor_tests.h"
#include "gnc/estimation/sun_sensor_to_vector.h"
#include "macros.h"

void test_sun_pyramid_reading(slate_t *slate)
{
    LOG_INFO("Testing sun_pyramid_reading...");

    // Initialize
    for (int i = 0; i < 4; ++i)
    {
        slate->sun_sensors_intensities[i] = 1.0f;
    }

    for (int i = 4; i < 8; ++i)
    {
        slate->sun_sensors_intensities[i] = 0.0f;
    }

    // slate->sun_sensors_intensities[0] = {1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f,
    // 0.0f, 0.0f, 0.0f, 0.0f};

    sun_sensors_to_vector(slate);
    LOG_INFO("Sun vector in body frame: %f, %f, %f", slate->sun_vector_body[0],
             slate->sun_vector_body[1], slate->sun_vector_body[2]);
    // should be [1 0 0]
    LOG_INFO("sun_pyramid_reading testing successful! :)");
}

void test_sun_sensor_real_values(slate_t *slate)
{
    // Compute sun vector from current sensor readings
    sun_sensors_to_vector(slate);

    // Log computed sun vector
    LOG_DEBUG("Sun vector: [%.6f, %.6f, %.6f]", slate->sun_vector_body.x,
              slate->sun_vector_body.y, slate->sun_vector_body.z);
}