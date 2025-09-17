/**
 * @author Lundeen Cahilly, Chen Li, and Tactical Cinderblock
 * @date 2025-08-24
 */

#pragma once

#include "linalg.h"
#include "slate.h"
#include "constants.h"

void sun_sensors_to_vector(slate_t *slate);

// External access to sensor configuration for testing
extern const float sensor_normals[NUM_SUN_SENSORS][3];