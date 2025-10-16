/**
 * @author Lundeen Cahilly, Chen Li, Tactical Cinderblock
 * @date 2025-08-24
 */

#pragma once

#include "constants.h"
#include "linalg.h"
#include "slate.h"

using namespace linalg::aliases;

void sun_sensors_to_vector(slate_t *slate);

#ifdef TEST
void test_sun_sensor_cases(slate_t *slate);
void test_sun_sensor_monte_carlo(slate_t *slate);
#endif