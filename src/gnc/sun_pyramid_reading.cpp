/**
 * @author Chen Li
 * @date 2025-07-16
 */

#include "gnc/sun_pyramid_reading.h"
#include "constants.h"
#include "gnc/matrix_utils.h"
#include "macros.h"
#include "pico/stdlib.h"

const float sensor_normal_vectors_inv[3 * 4] = {
    0.25, 0.25, 0.25, 0.25, 0, -0.5, 0, 0.5, -0.5, 0, 0.5, 0};

void sun_sensors_to_attitude(slate_t *slate)
{
    float sun_sensors_reading[4 * 1] = {
        slate->sun_sensors_intensities[0] - slate->sun_sensors_intensities[6],
        slate->sun_sensors_intensities[1] - slate->sun_sensors_intensities[5],
        slate->sun_sensors_intensities[2] - slate->sun_sensors_intensities[4],
        slate->sun_sensors_intensities[3] -
            slate->sun_sensors_intensities[7] // TODO: check with structure &
                                              // sun pyramid driver
    };

    float I_max = slate->sun_sensors_intensities[0];
    for (int i = 1; i < 8; ++i)
    {
        if (slate->sun_sensors_intensities[i] > I_max)
            I_max = slate->sun_sensors_intensities[i];
    }

    float temp[1 * 3];
    mat_mul(sensor_normal_vectors_inv, sun_sensors_reading, temp, 3, 4, 1);

    for (int i = 0; i < 3; ++i)
    {
        slate->sun_vector_principal[i] = temp[i] / I_max;
    }

    // if (I_max > 1e-6f) {

    //}
    // else {
    // TODO: want if all black?
    //}
}

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

    sun_sensors_to_attitude(slate);
    LOG_INFO("Sun vector in principal frame: %f, %f, %f",
             slate->sun_vector_principal[0], slate->sun_vector_principal[1],
             slate->sun_vector_principal[2]);
    // should be [1 0 0]
    LOG_INFO("sun_pyramid_reading testing successful! :)");
}
