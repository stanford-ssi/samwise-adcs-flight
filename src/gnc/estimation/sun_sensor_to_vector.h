/**
 * @author Lundeen Cahilly, Chen Li, Tactical Cinderblock
 * @date 2025-08-24
 */

#pragma once

#include "linalg.h"
using namespace linalg::aliases;

#include "drivers/sun_sensors/sun_sensors.h"

bool sun_sensors_to_vector(sun_sensor_data_t *sun_sensor, 
        float3 *sun_vector_body);

#ifdef TEST
// void test_sun_sensor_cases(slate_t *slate);
// void test_sun_sensor_monte_carlo(slate_t *slate);
#endif
