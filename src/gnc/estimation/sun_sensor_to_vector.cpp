/**
 * @author Lundeen Cahilly, Matthew Musson, Chen Li, TacCin
 * @date 2025-09-14
 * Convert sun sensor readings to sun vector :3
 */

#include "sun_sensor_to_vector.h"
#include "constants.h"
#include "gnc/utils/matrix_utils.h"
#include "macros.h"
#include "pico/stdlib.h"

// ========================================================================
//      CONSTANTS AND PARAMETERS
// ========================================================================

static constexpr uint16_t ACTIVE_THRESHOLD =
    500; // TODO: determine what makes sense for our system in LEO (0.5 * max i
         // think?) given the Earth's reflected light. this will limit our
         // effective FOV for each sensor

static constexpr uint16_t SATURATION_VALUE =
    3102; // max clipped value for sun sensors due to differing ref voltages

static constexpr int NUM_OPPOSITE_PAIRS = 8;

static constexpr int OPPOSITE_PAIRS[NUM_OPPOSITE_PAIRS][2] = {
    {0, 6},   {1, 5},  {2, 4}, {3, 7}, // pyramid opposites
    {8, 10},  {9, 11},                 // Y+ vs Y- pairs
    {12, 14}, {13, 15}                 // Z+ vs Z- pairs
};

static constexpr int MAX_ITERATIONS = 1000;
static constexpr float INLIER_THRESHOLD =
    0.1f; // threshold for considering a sensor an inlier TODO: tune this value

// ========================================================================
//      FORWARD DECLARATIONS
// ========================================================================

int valid_sensors(slate_t *slate, bool *exclude_sensors);
bool ransac_sun_vector(slate_t *slate, bool *exclude_sensors,
                       float3 &best_sun_vector, int &best_inlier_count);

// ========================================================================
//      SUN VECTOR ESTIMATION (main)
// ========================================================================

void sun_sensors_to_vector(slate_t *slate)
{
    /* Create matrix of normals weighted by intensity, and run these through
     * RANSAC (Random Sample Consensus) Sets sun vector to valid in SLATE if
     * RANSAC successful, else sets invalid
     */
    const float epsilon = 1e-6f;
    bool exclude_sensors[NUM_SUN_SENSORS] = {
        false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false};

    // Check if sun sensors are valid or not
    int valid_count = valid_sensors(slate, exclude_sensors);

    // Underdetermined case - set to zero and flag as invalid
    if (valid_count < 3)
    {
        slate->sun_vector_body = {0.0f, 0.0f, 0.0f};
        slate->sun_vector_valid = false;
        return;
    }

    // Use RANSAC sun vector estimation
    float3 sun_vector;
    int inlier_count;

    if (!ransac_sun_vector(slate, exclude_sensors, sun_vector, inlier_count))
    {
        slate->sun_vector_body = {0.0f, 0.0f, 0.0f};
        slate->sun_vector_valid = false;
        return;
    }
    slate->sun_vector_body = sun_vector;
    slate->sun_vector_valid = true;
    return;
}

// ========================================================================
//      SENSOR VALIDATION AND RANSAC
// ========================================================================

/**
 * @brief Validate sun sensor readings by checking for saturation and
 * opposite sensor activation
 *
 * @param slate Pointer to the ADCS slate structure
 * @param exclude_sensors output: array indicating which sensors are invalid
 * @return int Number of valid sensors
 */
int valid_sensors(slate_t *slate, bool *exclude_sensors)
{
    // Check for saturation
    for (int i = 0; i < NUM_SUN_SENSORS; i++)
    {
        uint16_t intensity = slate->sun_sensor_intensities[i];
        if (intensity >= SATURATION_VALUE)
        {
            exclude_sensors[i] = true;
        }
    }

    // Exclude if below threshold
    for (int i = 0; i < NUM_SUN_SENSORS; i++)
    {
        uint16_t intensity = slate->sun_sensor_intensities[i];
        if (intensity < ACTIVE_THRESHOLD)
        {
            exclude_sensors[i] = true;
        }
    }

    // Exclude opposite sensor pairs if both are active
    // (physically impossible - one must be faulty)
    for (int i = 0; i < NUM_OPPOSITE_PAIRS; i++)
    {
        int s1 = OPPOSITE_PAIRS[i][0];
        int s2 = OPPOSITE_PAIRS[i][1];

        // Skip if either sensor already excluded
        if (exclude_sensors[s1] || exclude_sensors[s2])
        {
            continue;
        }

        uint16_t intensity1 = slate->sun_sensor_intensities[s1];
        uint16_t intensity2 = slate->sun_sensor_intensities[s2];

        // If both are active, exclude both (we don't know which is faulty)
        if (intensity1 >= ACTIVE_THRESHOLD && intensity2 >= ACTIVE_THRESHOLD)
        {
            exclude_sensors[s1] = true;
            exclude_sensors[s2] = true;
        }
    }

    // Count valid sensors
    int valid_count = 0;
    for (int i = 0; i < NUM_SUN_SENSORS; i++)
    {
        if (!exclude_sensors[i])
        {
            valid_count++;
        }
    }

    return valid_count;
}

/**
 * @brief RANSAC implementation for robust sun vector estimation
 *
 * @param slate Pointer to the ADCS slate structure
 * @param exclude_sensors array indicating which sensors to exclude
 * @param best_sun_vector output: estimated sun vector
 * @param best_inlier_count output: number of inliers for best estimate
 *
 * @return true if successful, false if failed
 */
bool ransac_sun_vector(slate_t *slate, bool *exclude_sensors,
                       float3 &best_sun_vector, int &best_inlier_count)
{
    // Build list of valid sensor indices
    int valid_indices[NUM_SUN_SENSORS];
    int n_sensors = 0;
    for (int i = 0; i < NUM_SUN_SENSORS; i++)
    {
        if (!exclude_sensors[i])
        {
            valid_indices[n_sensors] = i;
            n_sensors++;
        }
    }

    if (n_sensors < 3)
    {
        return false;
    }

    best_inlier_count = 0;
    best_sun_vector = {0, 0, 0};

    // Try different combinations of 3 sensors
    for (int iter = 0;
         iter < MAX_ITERATIONS &&
         iter < (n_sensors * (n_sensors - 1) * (n_sensors - 2) / 6);
         iter++)
    {
        // Sample 3 unique sensors from valid set
        int idx0 = iter % n_sensors;
        int idx1 = (iter + 1) % n_sensors;
        int idx2 = (iter + 2) % n_sensors;

        // Make sure indices are unique
        if (idx0 == idx1 || idx1 == idx2 || idx0 == idx2)
        {
            continue;
        }

        // Get actual sensor indices
        int sensor_indices[3] = {valid_indices[idx0], valid_indices[idx1],
                                 valid_indices[idx2]};

        // Build 3x3 system: A * sun_vector = b
        float N[9]; // 3x3 matrix
        float b[3]; // RHS vector

        for (int i = 0; i < 3; i++)
        {
            int sensor_idx = sensor_indices[i];
            N[i * 3 + 0] = SUN_SENSOR_NORMALS[sensor_idx][0];
            N[i * 3 + 1] = SUN_SENSOR_NORMALS[sensor_idx][1];
            N[i * 3 + 2] = SUN_SENSOR_NORMALS[sensor_idx][2];
            b[i] =
                static_cast<float>(slate->sun_sensor_intensities[sensor_idx]);
        }

        // Check for singularity by computing determinant
        float det = N[0] * (N[4] * N[8] - N[5] * N[7]) -
                    N[1] * (N[3] * N[8] - N[5] * N[6]) +
                    N[2] * (N[3] * N[7] - N[4] * N[6]);

        if (fabs(det) < 1e-6f)
        {
            continue; // Skip if matrix is near-singular
        }

        // Solve 3x3 system using matrix inversion
        float N_inv[9];
        mat_inverse(N, N_inv, 3);

        // Compute sun vector candidate
        float sun_candidate[3];
        mat_mul(N_inv, b, sun_candidate, 3, 3, 1);

        float3 sun_vec = {sun_candidate[0], sun_candidate[1], sun_candidate[2]};

        // Normalize to unit vector
        float magnitude = length(sun_vec);
        if (magnitude < 1e-6f)
        {
            continue; // Skip if magnitude too small
        }
        sun_vec = sun_vec / magnitude;

        // Count inliers across all valid sensors
        int inlier_count = 0;
        for (int i = 0; i < n_sensors; i++)
        {
            int sensor_idx = valid_indices[i];
            float3 normal = {SUN_SENSOR_NORMALS[sensor_idx][0],
                             SUN_SENSOR_NORMALS[sensor_idx][1],
                             SUN_SENSOR_NORMALS[sensor_idx][2]};
            float expected = fmax(0.0f, dot(sun_vec, normal));
            float actual =
                static_cast<float>(slate->sun_sensor_intensities[sensor_idx]) /
                magnitude;

            float error = fabs(expected - actual);
            if (error < INLIER_THRESHOLD)
            {
                inlier_count++;
            }
        }

        // Update best solution if this one has more inliers
        if (inlier_count > best_inlier_count)
        {
            best_inlier_count = inlier_count;
            best_sun_vector = sun_vec;
        }
    }
    return best_inlier_count >
           3; // Need greater than 3 inliers for valid solution
}

#ifdef TEST
// TODO: add saturation test case
// Gaussian noise standard deviation for sensor readings
const float SENSOR_NOISE_STDDEV = 10.0f;    // Set to 0.0f for no noise
const float FLOATING_NOISE_STDDEV = 500.0f; // High noise for floating sensors

// Sensor failure modes
enum failure_type
{
    NONE = 0,    // Normal operation
    LOW = 1,     // Always reads 0 (dead sensor)
    FLOATING = 2 // Stuck at ~half clip value with high noise
};

// Global test cases used by both functions
struct test_case
{
    float3 sun_dir;
    bool occluded[NUM_SUN_SENSORS];
    failure_type failures[NUM_SUN_SENSORS];
    const char *description;
};

const struct test_case test_cases[] = {
    // No occlusion cases
    {{1.0f, 0.0f, 0.0f},
     {false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false},
     {NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,
      NONE, NONE, NONE, NONE},
     "Sun +X, no occlusion"},

    {{0.0f, 1.0f, 1.0f},
     {false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false},
     {NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,
      NONE, NONE, NONE, NONE},
     "Sun +Y and +Z, no occlusion"},

    // No occlusions but underdetermined
    {{0.0f, 1.0f, 0.0f},
     {false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false},
     {NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,
      NONE, NONE, NONE, NONE},
     "Sun +Y, no occlusion but underdetermined"},

    // Multiple sensor occlusions
    {{3.15889f, -6.26579f, 3.17831f},
     {true, false, false, true, true, true, false, false, false, false, false,
      false, false, false, false, false},
     {NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,
      NONE, NONE, NONE, NONE},
     "4 occluded case (2 unique active)"},

    // Test cases with sensor failures
    {{1.0f, 0.0f, 0.0f},
     {false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false},
     {NONE, LOW, FLOATING, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,
      NONE, NONE, NONE, NONE},
     "Sun +X with sensor 1 LOW, sensor 2 FLOATING"},

    {{0.0f, 1.0f, 0.2f},
     {false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false},
     {LOW, NONE, NONE, FLOATING, LOW, NONE, NONE, NONE, NONE, NONE, NONE, NONE,
      NONE, NONE, NONE, NONE},
     "Sun +Y+Z with sensor 1 LOW, sensor 3 FLOATING, sensor 4 LOW, and three "
     "sensors nominal"},

    {{1.0f, 0.0f, 0.0f},
     {false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false},
     {NONE, FLOATING, FLOATING, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE,
      NONE, NONE, NONE, NONE, NONE},
     "Sun +X with sensor 1 FLOATING, sensor 2 FLOATING, two sensors nominal"},

    {{1.0f, 0.0f, 0.0f},
     {false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false},
     {FLOATING, FLOATING, FLOATING, FLOATING, FLOATING, FLOATING, FLOATING,
      FLOATING, FLOATING, FLOATING, FLOATING, FLOATING, FLOATING, FLOATING,
      FLOATING, FLOATING},
     "Sun +X with all sensors FLOATING"},

    {{1.0f, 0.0f, 0.0f},
     {false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false},
     {NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, FLOATING, FLOATING,
      FLOATING, FLOATING, FLOATING, FLOATING, FLOATING, FLOATING},
     "Sun +X with YZ sensors FLOATING, sun pyramids nominal"},
};

const int NUM_TEST_CASES = sizeof(test_cases) / sizeof(test_cases[0]);

// Helper function to generate Gaussian noise
float generate_gaussian_noise(float stddev);

void test_sun_sensor_cases(slate_t *slate)
{
    for (int case_num = 0; case_num < NUM_TEST_CASES; case_num++)
    {
        printf("=== Test Case %d: %s ===\n", case_num,
               test_cases[case_num].description);

        // Calculate theoretical sensor values and apply failures
        for (int i = 0; i < NUM_SUN_SENSORS; i++)
        {
            float final_value = 0.0f;

            // Handle failure modes first
            if (test_cases[case_num].failures[i] == LOW)
            {
                // LOW failure: always reads 0
                final_value = 0.0f;
            }
            else if (test_cases[case_num].failures[i] == FLOATING)
            {
                // FLOATING failure: stuck at ~half clip value with high noise
                final_value = SUN_SENSOR_CLIP_VALUE / 2.0f +
                              generate_gaussian_noise(FLOATING_NOISE_STDDEV);
            }
            else if (!test_cases[case_num].occluded[i])
            {
                // Normal operation: calculate theoretical value
                float dot =
                    SUN_SENSOR_NORMALS[i][0] * test_cases[case_num].sun_dir.x +
                    SUN_SENSOR_NORMALS[i][1] * test_cases[case_num].sun_dir.y +
                    SUN_SENSOR_NORMALS[i][2] * test_cases[case_num].sun_dir.z;

                float theoretical = (dot > 0) ? dot * 1e3 : 0.0f;

                // Add normal noise if enabled
                if (SENSOR_NOISE_STDDEV > 0.0f)
                {
                    theoretical += generate_gaussian_noise(SENSOR_NOISE_STDDEV);
                }

                final_value = theoretical;
            }
            else
            {
                // Occluded: reads 0
                final_value = 0.0f;
            }

            // Clamp to valid sensor range and convert to uint16_t
            slate->sun_sensor_intensities[i] = (uint16_t)fmaxf(
                0.0f, fminf((float)SUN_SENSOR_CLIP_VALUE, final_value));
        }

        // Run estimation
        sun_sensors_to_vector(slate);

        // Print sensor values
        printf("Sun sensor readings:\n");
        for (int i = 0; i < NUM_SUN_SENSORS; i++)
        {
            // Calculate theoretical value (pure projection + clipping to 0)
            float dot =
                SUN_SENSOR_NORMALS[i][0] * test_cases[case_num].sun_dir.x +
                SUN_SENSOR_NORMALS[i][1] * test_cases[case_num].sun_dir.y +
                SUN_SENSOR_NORMALS[i][2] * test_cases[case_num].sun_dir.z;
            float theoretical = fmaxf(0.0f, dot * 1e3);

            const char *status = "";
            if (test_cases[case_num].failures[i] == LOW)
            {
                status = " (LOW)";
            }
            else if (test_cases[case_num].failures[i] == FLOATING)
            {
                status = " (FLOATING)";
            }
            else if (test_cases[case_num].occluded[i])
            {
                status = " (OCCLUDED)";
            }

            printf("  Sensor %2d: actual=%4d, theoretical=%7.1f%s\n", i,
                   slate->sun_sensor_intensities[i], theoretical, status);
        }

        // Calculate angular error
        if (slate->sun_vector_valid)
        {
            // Normalize expected direction
            float3 expected_normalized =
                normalize(test_cases[case_num].sun_dir);

            float dot = slate->sun_vector_body.x * expected_normalized.x +
                        slate->sun_vector_body.y * expected_normalized.y +
                        slate->sun_vector_body.z * expected_normalized.z;

            // Clamp dot product to valid range for acos
            dot = fmaxf(-1.0f, fminf(1.0f, dot));
            float angle_error = acosf(dot) * RAD_TO_DEG;

            printf("Expected: [%.5f, %.5f, %.5f]\n",
                   test_cases[case_num].sun_dir.x,
                   test_cases[case_num].sun_dir.y,
                   test_cases[case_num].sun_dir.z);
            printf("Result:   [%.5f, %.5f, %.5f]\n", slate->sun_vector_body.x,
                   slate->sun_vector_body.y, slate->sun_vector_body.z);
            printf("Angular error: %.5f degrees\n", angle_error);
        }
        else
        {
            printf("No sun sensor updated! \n");
        }
        printf("\n");
    }
}

void test_sun_sensor_monte_carlo(slate_t *slate)
{
    const int NUM_MONTE_CARLO_RUNS = 5000;

    // Select which test cases to run Monte Carlo on
    int selected_cases[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    int num_cases = sizeof(selected_cases) / sizeof(selected_cases[0]);

    printf("=== Monte Carlo Analysis (%d runs per case) ===\n",
           NUM_MONTE_CARLO_RUNS);

    for (int case_idx = 0; case_idx < num_cases; case_idx++)
    {
        int case_num = selected_cases[case_idx]; // Get actual test case index
        printf("Case %d: %s\n", case_num, test_cases[case_num].description);

        float total_error = 0.0f;
        float min_error = INFINITY;
        float max_error = -INFINITY;
        int successful_runs = 0;
        int failed_runs = 0;

        for (int run = 0; run < NUM_MONTE_CARLO_RUNS; run++)
        {
            // Apply the same failure mode logic as in test_sun_sensor_cases
            for (int i = 0; i < NUM_SUN_SENSORS; i++)
            {
                float final_value = 0.0f;

                // Handle failure modes first
                if (test_cases[case_num].failures[i] == LOW)
                {
                    // LOW failure: always reads 0
                    final_value = 0.0f;
                }
                else if (test_cases[case_num].failures[i] == FLOATING)
                {
                    // FLOATING failure: stuck at ~half clip value with high
                    // noise
                    final_value =
                        SUN_SENSOR_CLIP_VALUE / 2.0f +
                        generate_gaussian_noise(FLOATING_NOISE_STDDEV);
                }
                else if (!test_cases[case_num].occluded[i])
                {
                    // Normal operation: calculate theoretical value
                    float dot = SUN_SENSOR_NORMALS[i][0] *
                                    test_cases[case_num].sun_dir.x +
                                SUN_SENSOR_NORMALS[i][1] *
                                    test_cases[case_num].sun_dir.y +
                                SUN_SENSOR_NORMALS[i][2] *
                                    test_cases[case_num].sun_dir.z;

                    float theoretical = (dot > 0) ? dot * 1e3 : 0.0f;

                    // Add normal noise if enabled
                    if (SENSOR_NOISE_STDDEV > 0.0f)
                    {
                        theoretical +=
                            generate_gaussian_noise(SENSOR_NOISE_STDDEV);
                    }

                    final_value = theoretical;
                }
                else
                {
                    // Occluded: reads 0
                    final_value = 0.0f;
                }

                // Clamp to valid sensor range and convert to uint16_t
                slate->sun_sensor_intensities[i] = (uint16_t)fmaxf(
                    0.0f, fminf((float)SUN_SENSOR_CLIP_VALUE, final_value));
            }

            // Run estimation
            sun_sensors_to_vector(slate);

            // Calculate angular error
            if (slate->sun_vector_valid)
            {
                // Normalize expected direction
                float3 expected_normalized =
                    normalize(test_cases[case_num].sun_dir);

                float dot = slate->sun_vector_body.x * expected_normalized.x +
                            slate->sun_vector_body.y * expected_normalized.y +
                            slate->sun_vector_body.z * expected_normalized.z;

                // Clamp dot product to valid range for acos
                dot = fmaxf(-1.0f, fminf(1.0f, dot));
                float angle_error = acosf(dot) * RAD_TO_DEG;

                total_error += angle_error;
                if (angle_error < min_error)
                    min_error = angle_error;
                if (angle_error > max_error)
                    max_error = angle_error;
                successful_runs++;
            }
            else
            {
                failed_runs++;
            }
        }

        // Print statistics
        if (successful_runs > 0)
        {
            float avg_error = total_error / successful_runs;
            float success_rate =
                (float)successful_runs / NUM_MONTE_CARLO_RUNS * 100.0f;
            printf("  Success rate: %.1f%% (%d/%d)\n", success_rate,
                   successful_runs, NUM_MONTE_CARLO_RUNS);
            printf("  Angular error - Mean: %.6f deg, Min: %.6f deg, Max: %.6f "
                   "deg\n",
                   avg_error, min_error, max_error);
        }
        else
        {
            printf("  No sun vector: 0%% success rate\n");
        }
        printf("\n");
    }
}

float generate_gaussian_noise(float stddev)
{
    static float spare = 0.0f;
    static bool has_spare = false;

    if (has_spare)
    {
        has_spare = false;
        return spare * stddev;
    }

    has_spare = true;
    float u, v, s;
    do
    {
        u = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
        v = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
        s = u * u + v * v;
    } while (s >= 1.0f || s == 0.0f);

    s = sqrtf(-2.0f * logf(s) / s);
    spare = v * s;
    return u * s * stddev;
}
#endif