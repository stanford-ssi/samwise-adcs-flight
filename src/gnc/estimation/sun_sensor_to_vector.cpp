/**
 * @author Lundeen Cahilly, Tactical Cinderblock, Chen Li
 * @date 2025-09-14
 * 
 * Convert sun sensor readings to sun vector :3
 */

#include "sun_sensor_to_vector.h"
#include "constants.h"
#include "gnc/utils/matrix_utils.h"
#include "macros.h"
#include "pico/stdlib.h"

// Global storage for computed pseudoinverse (3xNUM_SUN_SENSORS)
static float sensor_pseudoinverse[3][NUM_SUN_SENSORS];
static bool pseudoinverse_computed = false;

// Shadow detection parameters
const float SHADOW_THRESHOLD = 0.05f;  // relative intensity threshold for shadow detection
const float MIN_WEIGHT = 0.01f;        // minimum weight for any sensor

float3 best_fit_occlusion(float sensor_readings[NUM_SUN_SENSORS]);
float get_occl_factor(const float *sensor_readings, const float *normals, int M, int N, float3& current_sun);

void sun_sensors_to_vector(slate_t *slate)
{
    // Use all sensor readings
    float sensor_readings[NUM_SUN_SENSORS];
    for (int i = 0; i < NUM_SUN_SENSORS; i++)
    {
        sensor_readings[i] =
            static_cast<float>(slate->sun_sensors_intensities[i]);
    }

    // Find max intensity for threshold checks
    float I_max = sensor_readings[0];
    for (int i = 1; i < NUM_SUN_SENSORS; i++)
    {
        if (sensor_readings[i] > I_max)
        {
            I_max = sensor_readings[i];
        }
    }

    // Check if we have enough light to make a measurement
    if (I_max < 1e-6f)
    {
        LOG_DEBUG("[sun_sensor_to_vector] No sun detected (max intensity = %.6f)", I_max);
        // No sun detected - set sun vector to zero and flag as invalid
        slate->sun_vector_body = {0, 0, 0};
        slate->sun_vector_valid = false;
        return;
    }

    // Check for saturation
    if (I_max >= SUN_SENSOR_CLIP_VALUE)
    {
        LOG_DEBUG("[sun_sensor_to_vector] Saturation detected in sun sensor readings");
        // Saturation detected - set sun vector to zero and flag as invalid
        slate->sun_vector_body = {0, 0, 0};
        slate->sun_vector_valid = false;
        return;
    }

    // Normalize sun_sensor_readings AFTER checking for saturation
    float norm = 0;
    for (int i = 0; i < NUM_SUN_SENSORS; i++)
    {
        norm += sensor_readings[i] * sensor_readings[i];
    }
    norm = sqrtf(norm);
    for (int i = 0; i < NUM_SUN_SENSORS; i++)
    {
        sensor_readings[i] /= norm;
    }

    // TODO: comment
    float3 sun_vector_best = best_fit_occlusion(sensor_readings);

    // Normalize to unit vector
    float3 sun_vector = normalize(sun_vector_best);
    float magnitude = length(sun_vector_best);
    LOG_INFO("Sun vector magnitude: %.6f", magnitude);

    if (magnitude > 1e-6f)
    {
        slate->sun_vector_body = sun_vector;
        slate->sun_vector_valid = true;
        return;
    }
    else
    {
        // Degenerate case - set to zero and flag as invalid
        slate->sun_vector_body = {0.0f, 0.0f, 0.0f};
        slate->sun_vector_valid = false;
        LOG_INFO("Sun vector solution failed, magnitude too small: %.6f", magnitude);
        return;
    }
}

float3 best_fit_occlusion(float sensor_readings[NUM_SUN_SENSORS]) 
{
    // sensor * N^-1 * N = sensor
    // sensor_readings MUST BE NORMALIZED (:3)

    float sensor_readings_unique[12];
    for (int i = 0; i < 8; i++)
    {
        sensor_readings_unique[i] = sensor_readings[i];
    }

    // Averaging redundant +Y/-Y sensors
    sensor_readings_unique[8] = (sensor_readings[8] + sensor_readings[9]) / 2;
    sensor_readings_unique[9] = (sensor_readings[10] + sensor_readings[11]) / 2;
    
    // Including +Z/-Z sensors
    sensor_readings_unique[10] = (sensor_readings[12] + sensor_readings[13]) / 2;
    sensor_readings_unique[11] = (sensor_readings[14] + sensor_readings[15]) / 2;

    int normals_idx[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14};
    float target_occl_factor = 1e-9f;
    float min_occl_factor; // Store best 
    float3 best_sun, current_sun;

    int8_t occl_cases[30][4] = {
        {0, -1, -1, -1},
        {1, -1, -1, -1},
        {2, -1, -1, -1},
        {3, -1, -1, -1},
        {4, -1, -1, -1},
        {5, -1, -1, -1},
        {6, -1, -1, -1},
        {7, -1, -1, -1},
        {8, -1, -1, -1}, // Define redundant sensors as one 
        {9, -1, -1, -1},
        {0,  1, -1, -1},
        {0,  3, -1, -1},
        {1,  2, -1, -1},
        {2,  3, -1, -1},
        {4,  5, -1, -1},
        {4,  7, -1, -1},
        {5,  6, -1, -1},
        {6,  7, -1, -1},
        {0,  2,  3, -1},
        {4,  6,  7, -1},
        {0,  3,  5, -1},
        {1,  4,  7, -1},
        {2,  3,  5, -1},
        {1,  6,  7, -1},
        {0,  2,  3,  5},
        {1,  4,  6,  7},
        {0,  3,  4,  5},
        {2,  3,  5,  6},
        {0,  1,  4,  7},
        {6,  7,  1,  2},
    };

    // Normals is normal vectors minus excluded; signals is readings minus excluded
    float signals[12];
    float normals[12][3];

    // Case 1: no occlusion
    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 3; j++) {
            normals[i][j] = SUN_SENSOR_NORMALS[normals_idx[i]][j];
        }
    }
    // temp debug
    int bestoccl = 0;

    min_occl_factor = get_occl_factor(sensor_readings_unique, (float*)normals, 12, 3, current_sun);
    best_sun = current_sun;
    if (min_occl_factor < target_occl_factor) return {current_sun[0], current_sun[1], current_sun[2]}; 
    LOG_INFO("[sun_sensor_to_vector] No occlusion factor: %.6f", min_occl_factor);

    // Case 2: with occlusions
    for (int a = 0; a < 10; a++) {
        uint8_t k = 0, e = 0; // k for included, e for excluded index in case
        for (uint8_t i = 0; i < 12; i++) {
            // All cases set in increasing order
            if (e < 4 && i == occl_cases[a][e]) {
                e++;
                continue;
            }
            for (int j = 0; j < 3; j++) {
                normals[k][j] = SUN_SENSOR_NORMALS[normals_idx[i]][j];
            }
            signals[k] = sensor_readings_unique[i];
            k++; // Length of the included sensors (by the end of the loop)
        }
        float occl_factor = get_occl_factor(signals, (float*)normals, k, 3, current_sun);
        if (occl_factor < min_occl_factor) {
            min_occl_factor = occl_factor;
            for (int i = 0; i < 3; i++) best_sun[i] = current_sun[i];
            // temp debug
            bestoccl = a + 1;
        }
        if (min_occl_factor < target_occl_factor) return {best_sun[0], best_sun[1], best_sun[2]};
    }

    LOG_INFO("Best occlusion factor: %.6f", min_occl_factor);
    LOG_INFO("Best occlusion case: %d", bestoccl);

    // Default fallback return
    return {best_sun[0], best_sun[1], best_sun[2]};
}

/**
 * Check for sensor occlusion by computing reconstruction residual using
 * sensor_readings * N^-1 * N and measuring fit quality
 *
 * @param sensor_readings
 * @param normals
 * @param M
 * @param N
 */
float get_occl_factor(const float *sensor_readings, const float *normals, int M, int N, float3& current_sun) {
    // Normalize sensor readings
    float readings_norm = 0;
    for (int i = 0; i < M; i++) {
        readings_norm += sensor_readings[i] * sensor_readings[i];
    }
    readings_norm = sqrt(readings_norm);

    float normalized_readings[M];
    for (int i = 0; i < M; i++) {
        normalized_readings[i] = sensor_readings[i] / readings_norm;
    }

    // Compute pseudoinverse: N^-1
    float sensor_pseudoinverse[N * M];
    mat_pseudoinverse(normals, sensor_pseudoinverse, M, N);
    
    // Compute N^-1 * normalized_readings
    float sun_vector[N];
    mat_mul(sensor_pseudoinverse, normalized_readings, sun_vector, N, M, 1);

    // Normalize sun vector
    float sun_norm = 0;
    for (int i = 0; i < N; i++) {
        sun_norm += sun_vector[i] * sun_vector[i];
    }
    sun_norm = sqrt(sun_norm);

    for (int i = 0; i < N; i++) {
        sun_vector[i] /= sun_norm;
    }

    // Reconstruct: N * normalized_sun_vector
    float reconstructed[M];
    mat_mul(normals, sun_vector, reconstructed, M, N, 1);

    // Clamp reconstructed values to zero (sun sensors can't have negative readings)
    for (int i = 0; i < M; i++) {
        if (reconstructed[i] < 0) reconstructed[i] = 0;
    }

    // Normalize reconstructed readings
    float recon_norm = 0;
    for (int i = 0; i < M; i++) {
        recon_norm += reconstructed[i] * reconstructed[i];
    }
    recon_norm = sqrt(recon_norm);
    for (int i = 0; i < M; i++) {
        reconstructed[i] /= recon_norm;
    }

    // Return residual (difference from normalized readings)
    float residual = 0;
    for (int i = 0; i < M; i++) {
        float diff = normalized_readings[i] - reconstructed[i];
        residual += diff * diff;
    }
    float norm = sqrt(residual);
    
    // Store current sun vector
    for (int i = 0; i < 3; i++) {
       current_sun[i] = sun_vector[i];
    }

    return norm;
}