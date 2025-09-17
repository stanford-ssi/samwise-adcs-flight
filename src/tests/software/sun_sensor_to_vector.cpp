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

// Gaussian noise standard deviation for sensor readings
const float SENSOR_NOISE_STDDEV = 10.0f;  // Set to 0.0f for no noise

void test_sun_sensor_cases(slate_t *slate)
{
    // Test case matrix: [sun_direction][occluded_sensors]
    struct {
        float3 sun_dir;
        bool occluded[NUM_SUN_SENSORS];
        const char* description;
    } test_cases[] = {
        // No occlusion cases
        {{1.0f, 0.0f, 0.0f}, {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}, "Sun +X, no occlusion"},
        {{0.0f, 1.0f, 0.2f}, {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}, "Sun +Y and +Z, no occlusion"},

        // No occlusions but underdetermined
        {{0.0f, 1.0f, 0.0f}, {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}, "Sun +Y, no occlusion but underdetermined"},

        // Multiple sensor occlusions
        {{3.15889f, -6.26579f, 3.17831f}, {true, false, false, true, true, true, false, false, false, false, false, false, false, false, false, false}, "4 occluded case (2 unique active)"},
    };

    int num_cases = sizeof(test_cases) / sizeof(test_cases[0]);

    for (int case_num = 0; case_num < num_cases; case_num++) {
        printf("=== Test Case %d: %s ===\n", case_num, test_cases[case_num].description);

        // Calculate theoretical sensor values with Gaussian noise
        for (int i = 0; i < NUM_SUN_SENSORS; i++) {
            float dot = SUN_SENSOR_NORMALS[i][0] * test_cases[case_num].sun_dir.x +
                        SUN_SENSOR_NORMALS[i][1] * test_cases[case_num].sun_dir.y +
                        SUN_SENSOR_NORMALS[i][2] * test_cases[case_num].sun_dir.z;

            float theoretical = (dot > 0) ? dot * 1e3 : 0.0f;

            // Add Gaussian noise if enabled
            if (SENSOR_NOISE_STDDEV > 0.0f && !test_cases[case_num].occluded[i]) {
                // Simple Box-Muller transform for Gaussian noise
                static float spare = 0.0f;
                static bool has_spare = false;

                float noise;
                if (has_spare) {
                    noise = spare * SENSOR_NOISE_STDDEV;
                    has_spare = false;
                } else {
                    float u = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
                    float v = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
                    float s = u * u + v * v;
                    if (s >= 1.0f || s == 0.0f) {
                        // Retry if outside unit circle
                        noise = 0.0f;
                    } else {
                        s = sqrtf(-2.0f * logf(s) / s);
                        noise = u * s * SENSOR_NOISE_STDDEV;
                        spare = v * s;
                        has_spare = true;
                    }
                }
                theoretical += noise;
            }

            slate->sun_sensors_intensities[i] = test_cases[case_num].occluded[i] ? 0 : (uint16_t)fmaxf(0.0f, theoretical);
        }

        // Run estimation
        sun_sensors_to_vector(slate);

        // Calculate angular error
        if (slate->sun_vector_valid) {
            // Normalize expected direction
            float3 expected_normalized = normalize(test_cases[case_num].sun_dir);

            float dot = slate->sun_vector_body.x * expected_normalized.x +
                        slate->sun_vector_body.y * expected_normalized.y +
                        slate->sun_vector_body.z * expected_normalized.z;

            // Clamp dot product to valid range for acos
            dot = fmaxf(-1.0f, fminf(1.0f, dot));
            float angle_error = acosf(dot) * RAD_TO_DEG;

            printf("[sun_sensor_to_vector_test] Expected: [%.5f, %.5f, %.5f]\n", test_cases[case_num].sun_dir.x, test_cases[case_num].sun_dir.y, test_cases[case_num].sun_dir.z);
            printf("[sun_sensor_to_vector_test] Result:   [%.5f, %.5f, %.5f]\n", slate->sun_vector_body.x, slate->sun_vector_body.y, slate->sun_vector_body.z);
            printf("[sun_sensor_to_vector_test] Angular error: %.5f degrees\n", angle_error);
        } else {
            printf("[sun_sensor_to_vector_test] No sun sensor updated! \n");
        }
        printf("\n");
    }
}

void test_sun_sensor_monte_carlo(slate_t *slate)
{
    const int NUM_MONTE_CARLO_RUNS = 1000;

    // Same test cases as above
    struct {
        float3 sun_dir;
        bool occluded[NUM_SUN_SENSORS];
        const char* description;
    } test_cases[] = {
        // No occlusion cases
        {{1.0f, 0.0f, 0.0f}, {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}, "Sun +X, no occlusion"},
        {{0.0f, 1.0f, 0.2f}, {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}, "Sun +Y and +Z, no occlusion"},
    };

    int num_cases = sizeof(test_cases) / sizeof(test_cases[0]);

    printf("=== Monte Carlo Analysis (%d runs per case) ===\n", NUM_MONTE_CARLO_RUNS);

    for (int case_num = 0; case_num < num_cases; case_num++) {
        printf("\nCase %d: %s\n", case_num, test_cases[case_num].description);

        float total_error = 0.0f;
        int successful_runs = 0;
        int failed_runs = 0;

        for (int run = 0; run < NUM_MONTE_CARLO_RUNS; run++) {
            // Calculate theoretical sensor values with Gaussian noise
            for (int i = 0; i < NUM_SUN_SENSORS; i++) {
                float dot = SUN_SENSOR_NORMALS[i][0] * test_cases[case_num].sun_dir.x +
                            SUN_SENSOR_NORMALS[i][1] * test_cases[case_num].sun_dir.y +
                            SUN_SENSOR_NORMALS[i][2] * test_cases[case_num].sun_dir.z;

                float theoretical = (dot > 0) ? dot * 1e3 : 0.0f;

                // Add Gaussian noise if enabled
                if (SENSOR_NOISE_STDDEV > 0.0f && !test_cases[case_num].occluded[i]) {
                    // Simple Box-Muller transform for Gaussian noise
                    static float spare = 0.0f;
                    static bool has_spare = false;

                    float noise;
                    if (has_spare) {
                        noise = spare * SENSOR_NOISE_STDDEV;
                        has_spare = false;
                    } else {
                        float u = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
                        float v = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
                        float s = u * u + v * v;
                        if (s >= 1.0f || s == 0.0f) {
                            // Retry if outside unit circle
                            noise = 0.0f;
                        } else {
                            s = sqrtf(-2.0f * logf(s) / s);
                            noise = u * s * SENSOR_NOISE_STDDEV;
                            spare = v * s;
                            has_spare = true;
                        }
                    }
                    theoretical += noise;
                }

                slate->sun_sensors_intensities[i] = test_cases[case_num].occluded[i] ? 0 : (uint16_t)fmaxf(0.0f, theoretical);
            }

            // Run estimation
            sun_sensors_to_vector(slate);

            // Calculate angular error
            if (slate->sun_vector_valid) {
                // Normalize expected direction
                float3 expected_normalized = normalize(test_cases[case_num].sun_dir);

                float dot = slate->sun_vector_body.x * expected_normalized.x +
                            slate->sun_vector_body.y * expected_normalized.y +
                            slate->sun_vector_body.z * expected_normalized.z;

                // Clamp dot product to valid range for acos
                dot = fmaxf(-1.0f, fminf(1.0f, dot));
                float angle_error = acosf(dot) * RAD_TO_DEG;

                total_error += angle_error;
                successful_runs++;
            } else {
                failed_runs++;
            }
        }

        // Print statistics
        if (successful_runs > 0) {
            float avg_error = total_error / successful_runs;
            float success_rate = (float)successful_runs / NUM_MONTE_CARLO_RUNS * 100.0f;
            printf("  Success rate: %.1f%% (%d/%d)\n", success_rate, successful_runs, NUM_MONTE_CARLO_RUNS);
            printf("  Average angular error: %.3f degrees\n", avg_error);
        } else {
            printf("  FAILED: 0%% success rate\n");
        }
        printf("\n");
    }
}