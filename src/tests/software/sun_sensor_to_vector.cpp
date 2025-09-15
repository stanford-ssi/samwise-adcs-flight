/**
 * @author Lundeen Cahilly and Tactical Cinderblock
 * @date 2025-09-14
 * 
 * Simple eclipse test - panels block pyramid sensors
 */

#include "sun_sensor_to_vector.h"
#include "gnc/estimation/sun_sensor_to_vector.h"
#include "constants.h"
#include "macros.h"
#include "linalg.h"

using namespace linalg::aliases;

void test_eclipse(slate_t *slate)
{
    float3 sun_dir = {3.15889f, -6.26579f, 3.17831f};
    sun_dir = normalize(sun_dir);

    // Occlusion table - which sensors are blocked
    bool occluded[NUM_SUN_SENSORS] = {
        true, false,  false, true,   // pyramid 1
        true, true,  false, false, // pyramid 2
        false, false,               // Y+ sensors
        false, false,               // Y- sensors
        false, false,               // Z+ sensors
        false, false                // Z- sensors
    };

    // Step 1: Calculate theoretical values (cosine relation)
    printf("Theoretical values for sun = [%.3f, %.3f, %.3f]:\n", sun_dir.x, sun_dir.y, sun_dir.z);
    for (int i = 0; i < NUM_SUN_SENSORS; i++)
    {
        float dot = SUN_SENSOR_NORMALS[i][0] * sun_dir.x +
                    SUN_SENSOR_NORMALS[i][1] * sun_dir.y +
                    SUN_SENSOR_NORMALS[i][2] * sun_dir.z;

        // Normalize by sensor normal magnitude
        float normal_mag = sqrtf(SUN_SENSOR_NORMALS[i][0] * SUN_SENSOR_NORMALS[i][0] +
                                SUN_SENSOR_NORMALS[i][1] * SUN_SENSOR_NORMALS[i][1] +
                                SUN_SENSOR_NORMALS[i][2] * SUN_SENSOR_NORMALS[i][2]);
        float normalized_dot = dot / normal_mag;

        float theoretical = (normalized_dot > 0) ? normalized_dot * 1e3 : 0.0f;

        printf("  Sensor %d: %0.2f %s\n", i, theoretical, occluded[i] ? "(OCCLUDED)" : "");

        // Step 2: Apply occlusion
        slate->sun_sensors_intensities[i] = occluded[i] ? 0 : (uint16_t)theoretical;
    }

    // Step 3: Run estimation
    sun_sensors_to_vector(slate);
    printf("Result: [%.2f, %.2f, %.2f] valid=%d\n",
           slate->sun_vector_body[0], slate->sun_vector_body[1], slate->sun_vector_body[2],
           slate->sun_vector_valid);

    // Find angular error between estimated and true sun vector
    float dot = slate->sun_vector_body.x * sun_dir.x +
                slate->sun_vector_body.y * sun_dir.y +
                slate->sun_vector_body.z * sun_dir.z;
    float angle_error = acosf(dot) * RAD_TO_DEG;
    printf("Angular error: %.2f degrees\n", angle_error);
}