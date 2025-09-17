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
const float FLOATING_NOISE_STDDEV = 500.0f;  // High noise for floating sensors

// Sensor failure modes
enum failure_type {
    NONE = 0,     // Normal operation
    LOW = 1,      // Always reads 0 (dead sensor)
    FLOATING = 2  // Stuck at ~half clip value with high noise
};

// Global test cases used by both functions
struct test_case {
    float3 sun_dir;
    bool occluded[NUM_SUN_SENSORS];
    failure_type failures[NUM_SUN_SENSORS];
    const char* description;
};

const struct test_case test_cases[] = {
    // No occlusion cases
    {{1.0f, 0.0f, 0.0f},
     {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false},
     {NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE},
     "Sun +X, no occlusion"},

    {{0.0f, 1.0f, 0.2f},
     {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false},
     {NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE},
     "Sun +Y and +Z, no occlusion"},

    // No occlusions but underdetermined
    {{0.0f, 1.0f, 0.0f},
     {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false},
     {NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE},
     "Sun +Y, no occlusion but underdetermined"},

    // Multiple sensor occlusions
    {{3.15889f, -6.26579f, 3.17831f},
     {true, false, false, true, true, true, false, false, false, false, false, false, false, false, false, false},
     {NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE},
     "4 occluded case (2 unique active)"},

    // Test cases with sensor failures
    {{1.0f, 0.0f, 0.0f},
     {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false},
     {NONE, LOW, FLOATING, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE},
     "Sun +X with sensor 1 LOW, sensor 2 FLOATING"},

    {{0.0f, 1.0f, 0.2f},
     {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false},
     {LOW, NONE, NONE, FLOATING, LOW, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE},
     "Sun +Y+Z with sensor 1 LOW, sensor 3 FLOATING, sensor 4 LOW, and three sensors nominal"},

    {{1.0f, 0.0f, 0.0f},
     {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false},
     {NONE, FLOATING, FLOATING, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE},
     "Sun +X with sensor 1 FLOATING, sensor 2 FLOATING, two sensors nominal"},
};

const int NUM_TEST_CASES = sizeof(test_cases) / sizeof(test_cases[0]);

// Helper function to generate Gaussian noise
float generate_gaussian_noise(float stddev);

void test_sun_sensor_cases(slate_t *slate)
{
    for (int case_num = 0; case_num < NUM_TEST_CASES; case_num++) {
        printf("=== Test Case %d: %s ===\n", case_num, test_cases[case_num].description);

        // Calculate theoretical sensor values and apply failures
        for (int i = 0; i < NUM_SUN_SENSORS; i++) {
            float final_value = 0.0f;

            // Handle failure modes first
            if (test_cases[case_num].failures[i] == LOW) {
                // LOW failure: always reads 0
                final_value = 0.0f;
            }
            else if (test_cases[case_num].failures[i] == FLOATING) {
                // FLOATING failure: stuck at ~half clip value with high noise
                final_value = SUN_SENSOR_CLIP_VALUE / 2.0f + generate_gaussian_noise(FLOATING_NOISE_STDDEV);
            }
            else if (!test_cases[case_num].occluded[i]) {
                // Normal operation: calculate theoretical value
                float dot = SUN_SENSOR_NORMALS[i][0] * test_cases[case_num].sun_dir.x +
                            SUN_SENSOR_NORMALS[i][1] * test_cases[case_num].sun_dir.y +
                            SUN_SENSOR_NORMALS[i][2] * test_cases[case_num].sun_dir.z;

                float theoretical = (dot > 0) ? dot * 1e3 : 0.0f;

                // Add normal noise if enabled
                if (SENSOR_NOISE_STDDEV > 0.0f) {
                    theoretical += generate_gaussian_noise(SENSOR_NOISE_STDDEV);
                }

                final_value = theoretical;
            }
            else {
                // Occluded: reads 0
                final_value = 0.0f;
            }

            // Clamp to valid sensor range and convert to uint16_t
            slate->sun_sensors_intensities[i] = (uint16_t)fmaxf(0.0f, fminf((float)SUN_SENSOR_CLIP_VALUE, final_value));
        }

        // Run estimation
        sun_sensors_to_vector(slate);

        // Print sensor values
        printf("Sun sensor readings:\n");
        for (int i = 0; i < NUM_SUN_SENSORS; i++) {
            // Calculate theoretical value (pure projection + clipping to 0)
            float dot = SUN_SENSOR_NORMALS[i][0] * test_cases[case_num].sun_dir.x +
                        SUN_SENSOR_NORMALS[i][1] * test_cases[case_num].sun_dir.y +
                        SUN_SENSOR_NORMALS[i][2] * test_cases[case_num].sun_dir.z;
            float theoretical = fmaxf(0.0f, dot * 1e3);

            const char* status = "";
            if (test_cases[case_num].failures[i] == LOW) {
                status = " (LOW)";
            } else if (test_cases[case_num].failures[i] == FLOATING) {
                status = " (FLOATING)";
            } else if (test_cases[case_num].occluded[i]) {
                status = " (OCCLUDED)";
            }

            printf("  Sensor %2d: actual=%4d, theoretical=%7.1f%s\n",
                    i, slate->sun_sensors_intensities[i], theoretical, status);
        }

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

            printf("Expected: [%.5f, %.5f, %.5f]\n", test_cases[case_num].sun_dir.x, test_cases[case_num].sun_dir.y, test_cases[case_num].sun_dir.z);
            printf("Result:   [%.5f, %.5f, %.5f]\n", slate->sun_vector_body.x, slate->sun_vector_body.y, slate->sun_vector_body.z);
            printf("Angular error: %.5f degrees\n", angle_error);
        } else {
            printf("No sun sensor updated! \n");
        }
        printf("\n");
    }
}

void test_sun_sensor_monte_carlo(slate_t *slate)
{
    const int NUM_MONTE_CARLO_RUNS = 5000;

    // Select which test cases to run Monte Carlo on
    int selected_cases[] = {0, 1, 2, 3, 4, 5, 6}; 
    int num_cases = sizeof(selected_cases) / sizeof(selected_cases[0]);

    printf("=== Monte Carlo Analysis (%d runs per case) ===\n", NUM_MONTE_CARLO_RUNS);

    for (int case_idx = 0; case_idx < num_cases; case_idx++) {
        int case_num = selected_cases[case_idx];  // Get actual test case index
        printf("Case %d: %s\n", case_num, test_cases[case_num].description);

        float total_error = 0.0f;
        int successful_runs = 0;
        int failed_runs = 0;

        for (int run = 0; run < NUM_MONTE_CARLO_RUNS; run++) {
            // Apply the same failure mode logic as in test_sun_sensor_cases
            for (int i = 0; i < NUM_SUN_SENSORS; i++) {
                float final_value = 0.0f;

                // Handle failure modes first
                if (test_cases[case_num].failures[i] == LOW) {
                    // LOW failure: always reads 0
                    final_value = 0.0f;
                }
                else if (test_cases[case_num].failures[i] == FLOATING) {
                    // FLOATING failure: stuck at ~half clip value with high noise
                    final_value = SUN_SENSOR_CLIP_VALUE / 2.0f + generate_gaussian_noise(FLOATING_NOISE_STDDEV);
                }
                else if (!test_cases[case_num].occluded[i]) {
                    // Normal operation: calculate theoretical value
                    float dot = SUN_SENSOR_NORMALS[i][0] * test_cases[case_num].sun_dir.x +
                                SUN_SENSOR_NORMALS[i][1] * test_cases[case_num].sun_dir.y +
                                SUN_SENSOR_NORMALS[i][2] * test_cases[case_num].sun_dir.z;

                    float theoretical = (dot > 0) ? dot * 1e3 : 0.0f;

                    // Add normal noise if enabled
                    if (SENSOR_NOISE_STDDEV > 0.0f) {
                        theoretical += generate_gaussian_noise(SENSOR_NOISE_STDDEV);
                    }

                    final_value = theoretical;
                }
                else {
                    // Occluded: reads 0
                    final_value = 0.0f;
                }

                // Clamp to valid sensor range and convert to uint16_t
                slate->sun_sensors_intensities[i] = (uint16_t)fmaxf(0.0f, fminf((float)SUN_SENSOR_CLIP_VALUE, final_value));
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
            printf("  Average angular error: %.6f degrees\n", avg_error);
        } else {
            printf("  No sun vector: 0%% success rate\n");
        }
        printf("\n");
    }
}

float generate_gaussian_noise(float stddev)
{
    static float spare = 0.0f;
    static bool has_spare = false;

    if (has_spare) {
        has_spare = false;
        return spare * stddev;
    }

    has_spare = true;
    float u, v, s;
    do {
        u = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
        v = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
        s = u * u + v * v;
    } while (s >= 1.0f || s == 0.0f);

    s = sqrtf(-2.0f * logf(s) / s);
    spare = v * s;
    return u * s * stddev;
}