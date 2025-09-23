/**
 * @author Lundeen Cahilly, Chen Li, and Tactical Cinderblock
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
const uint16_t ACTIVE_THRESHOLD = 50;

bool solve_sun_vector_ransac(float normals[][3], float signals[], int n_sensors, float3& best_sun_vector, int& best_inlier_count);
bool compute_sun_vector_ransac(slate_t *slate);
bool compute_sun_vector_pseudoinverse(slate_t *slate);

/**
 * Main function to convert sun sensor readings to sun vector.
 * Allows us to determine which algorithm to use in testing. 
 * TODO: remove this overhead
 * 
 * @param slate Pointer to the slate structure containing sun sensor data.
 */
void sun_sensors_to_vector(slate_t *slate)
{
    compute_sun_vector_ransac(slate);
    // compute_sun_vector_pseudoinverse(slate);
}

/**
 * Compute the sun vector using the pseudoinverse method, where 
 * sun_vector = N_pinv * I
 * 
 * @param slate Pointer to the slate structure containing sun sensor data.
 * @return true if the sun vector is successfully computed and valid, false otherwise.
 */
bool compute_sun_vector_pseudoinverse(slate_t *slate)
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

    // Check for saturation
    // TODO: figure out what to do in this case
    if (I_max >= SUN_SENSOR_CLIP_VALUE)
    {
        // Saturation detected - set sun vector to zero and flag as invalid
        slate->sun_vector_body = {0, 0, 0};
        slate->sun_vector_valid = false;
        return false;
    }

    // Define opposite sensor pairs that should never both be active
    const int opposite_pairs[][2] = {
        {0, 6}, {1, 5}, {2, 4}, {3, 7},    // pyramid opposites
        {8, 10}, {9, 11},                  // Y+ vs Y- pairs
        {12, 14}, {13, 15}                 // Z+ vs Z- pairs
    };
    const int num_opposite_pairs = 8;

    // Check if opposites are active at same time (not meant to be possible)
    bool exclude_sensor[NUM_SUN_SENSORS] = {false};
    for (int i = 0; i < num_opposite_pairs; i++) {
        int sensor1 = opposite_pairs[i][0];
        int sensor2 = opposite_pairs[i][1];
        
        if (sensor_readings[sensor1] > ACTIVE_THRESHOLD &&
            sensor_readings[sensor2] > ACTIVE_THRESHOLD) {
            exclude_sensor[sensor1] = true;
            exclude_sensor[sensor2] = true;
        }
    }

    // Create 6 differential readings by subtracting opposite sensors
    float sensor_readings_unique[6];

    // First 4 are pyramid differentials (positive - negative)
    sensor_readings_unique[0] = sensor_readings[0] - sensor_readings[6];  // 0-6 diff
    sensor_readings_unique[1] = sensor_readings[1] - sensor_readings[5];  // 1-5 diff
    sensor_readings_unique[2] = sensor_readings[2] - sensor_readings[4];  // 2-4 diff
    sensor_readings_unique[3] = sensor_readings[3] - sensor_readings[7];  // 3-7 diff

    // Y differential: average(Y+) - average(Y-)
    sensor_readings_unique[4] =
        (sensor_readings[8] + sensor_readings[9]) / 2 -
        (sensor_readings[10] + sensor_readings[11]) / 2;

    // Z differential: average(Z+) - average(Z-)
    sensor_readings_unique[5] =
        (sensor_readings[12] + sensor_readings[13]) / 2 -
        (sensor_readings[14] + sensor_readings[15]) / 2;

    // Use the positive direction normals for each differential pair
    int normals_idx[6] = {0, 1, 2, 3, 8, 12};

    // Only consider readings above threshold (differential readings can be negative)
    for (int i = 0; i < 6; i++)
    {
        if (fabs(sensor_readings_unique[i]) < ACTIVE_THRESHOLD)
        {
            sensor_readings_unique[i] = 0.0f;
        }
    }

    // create a matrix of normals and signals excluding zero sensors
    float normals[6][3];
    float signals[6];
    int valid_count = 0;
    for (int i = 0; i < 6; i++) {
        if (fabs(sensor_readings_unique[i]) > 1e-6f) {
            for (int j = 0; j < 3; j++) {
                normals[valid_count][j] = SUN_SENSOR_NORMALS[normals_idx[i]][j];
            }
            signals[valid_count] = sensor_readings_unique[i];
            valid_count++;
        }
    }

    // make sure top three are not zero
    if (valid_count < 3)
    {
        // Degenerate case - set to zero and flag as invalid
        slate->sun_vector_body = {0.0f, 0.0f, 0.0f};
        slate->sun_vector_valid = false;
        return false;
    }

    // compute psuedoinverse of valid sensors
    float sensor_pseudoinverse[3 * valid_count];
    mat_pseudoinverse((float*)normals, sensor_pseudoinverse, valid_count, 3);

    // Compute sun vector
    float sun_vector_3[3] = {0, 0, 0};
    mat_mul(sensor_pseudoinverse, signals, sun_vector_3, 3, valid_count, 1);
    float3 sun_vector = {sun_vector_3[0], sun_vector_3[1], sun_vector_3[2]};

    // Normalize to unit vector
    sun_vector = normalize(sun_vector);
    float magnitude = length(sun_vector);

    if (magnitude > 1e-6f)
    {
        slate->sun_vector_body = sun_vector;
        slate->sun_vector_valid = true;
        return true;
    }
    else
    {
        // Degenerate case - set to zero and flag as invalid
        slate->sun_vector_body = {0.0f, 0.0f, 0.0f};
        slate->sun_vector_valid = false;
        return false;
    }
}

/**
 * Compute the sun vector using the RANSAC method.
 * 
 * @param slate Pointer to the slate structure containing sun sensor data.
 * @return true if the sun vector is successfully computed and valid, false otherwise.
 */
bool compute_sun_vector_ransac(slate_t *slate)
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

    // Check for saturation
    // TODO: figure out what to do in this case
    if (I_max >= SUN_SENSOR_CLIP_VALUE)
    {
        // Saturation detected - set sun vector to zero and flag as invalid
        slate->sun_vector_body = {0, 0, 0};
        slate->sun_vector_valid = false;
        return false;
    }

    // Define opposite sensor pairs that should never both be active
    const int opposite_pairs[][2] = {
        {0, 6}, {1, 5}, {2, 4}, {3, 7},    // pyramid opposites
        {8, 10}, {9, 11},                  // Y+ vs Y- pairs
        {12, 14}, {13, 15}                 // Z+ vs Z- pairs
    };
    const int num_opposite_pairs = 8;

    // Mark sensors with impossible readings for exclusion
    bool exclude_sensor[NUM_SUN_SENSORS] = {false};
    for (int i = 0; i < num_opposite_pairs; i++) {
        int sensor1 = opposite_pairs[i][0];
        int sensor2 = opposite_pairs[i][1];
        if (sensor_readings[sensor1] > ACTIVE_THRESHOLD &&
            sensor_readings[sensor2] > ACTIVE_THRESHOLD) {
            exclude_sensor[sensor1] = true;
            exclude_sensor[sensor2] = true;
        }
    }

    // Define sensor readings in unique directions (excluding problematic sensors)
    float sensor_readings_unique[12];
    for (int i = 0; i < 8; i++)
    {
        sensor_readings_unique[i] = exclude_sensor[i] ? 0.0f : sensor_readings[i];
    }

    // Averaging redundant +Y/-Y sensors
    // TODO: do something smarter here because we're using RANSAC
    sensor_readings_unique[8] = (sensor_readings[8] + sensor_readings[9]) / 2;
    sensor_readings_unique[9] = (sensor_readings[10] + sensor_readings[11]) / 2;

    // Including +Z/-Z sensors (and averaging as above)
    sensor_readings_unique[10] = (sensor_readings[12] + sensor_readings[13]) / 2;
    sensor_readings_unique[11] = (sensor_readings[14] + sensor_readings[15]) / 2;

    int normals_idx[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14};

    // Only consider readings above threshold
    for (int i = 0; i < 12; i++)
    {
        if (sensor_readings_unique[i] < ACTIVE_THRESHOLD)
        {
            sensor_readings_unique[i] = 0.0f;
        }
    }

    // create a matrix of normals and signals excluding zero sensors
    float normals[12][3];
    float signals[12];
    int valid_count = 0;
    for (int i = 0; i < 12; i++) {
        if (sensor_readings_unique[i] > 1e-6f) {
            for (int j = 0; j < 3; j++) {
                normals[valid_count][j] = SUN_SENSOR_NORMALS[normals_idx[i]][j];
            }
            signals[valid_count] = sensor_readings_unique[i];
            valid_count++;
        }
    }

    // make sure top three are not zero
    if (valid_count < 3)
    {
        // Degenerate case - set to zero and flag as invalid
        slate->sun_vector_body = {0.0f, 0.0f, 0.0f};
        slate->sun_vector_valid = false;
        return false;
    }

    // Use RANSAC for robust sun vector estimation
    float3 sun_vector;
    int inlier_count;

    if (solve_sun_vector_ransac(normals, signals, valid_count, sun_vector, inlier_count))
    {
        slate->sun_vector_body = sun_vector;
        slate->sun_vector_valid = true;
        return true;
    }
    else
    {
        // RANSAC failed - set to zero and flag as invalid
        slate->sun_vector_body = {0.0f, 0.0f, 0.0f};
        slate->sun_vector_valid = false;
        return false;
    }
}

/**
 * @brief RANSAC implementation for robust sun vector estimation
 *
 * @param normals Array of sensor normal vectors [n_sensors][3]
 * @param signals Array of sensor readings [n_sensors]
 * @param n_sensors Number of valid sensors
 * @param best_sun_vector Output: best sun vector found
 * @param best_inlier_count Output: number of inliers for best solution
 * @return true if successful, false if failed
 */
bool solve_sun_vector_ransac(float normals[][3], float signals[], int n_sensors, float3& best_sun_vector, int& best_inlier_count)
{
    if (n_sensors < 3) {
        return false;
    }

    const int MAX_ITERATIONS = 100;
    const float INLIER_THRESHOLD = 0.2f; // threshold for considering a sensor an inlier

    best_inlier_count = 0;
    best_sun_vector = {0, 0, 0};

    // Try different combinations of 3 sensors
    for (int iter = 0; iter < MAX_ITERATIONS && iter < (n_sensors * (n_sensors-1) * (n_sensors-2) / 6); iter++) {
        // Sample 3 unique sensors
        int indices[3];
        indices[0] = iter % n_sensors;
        indices[1] = (iter + 1) % n_sensors;
        indices[2] = (iter + 2) % n_sensors;

        // Make sure indices are unique
        if (indices[0] == indices[1] || indices[1] == indices[2] || indices[0] == indices[2]) {
            continue;
        }

        // Build 3x3 system: A * sun_vector = b
        float N[9]; // 3x3 matrix
        float b[3]; // RHS vector

        for (int i = 0; i < 3; i++) {
            int sensor_idx = indices[i];
            N[i*3 + 0] = normals[sensor_idx][0];
            N[i*3 + 1] = normals[sensor_idx][1];
            N[i*3 + 2] = normals[sensor_idx][2];
            b[i] = signals[sensor_idx];
        }

        // Check for singularity by computing determinant
        float det = N[0] * (N[4] * N[8] - N[5] * N[7]) -
                   N[1] * (N[3] * N[8] - N[5] * N[6]) +
                   N[2] * (N[3] * N[7] - N[4] * N[6]);

        if (fabs(det) < 1e-6f) {
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
        if (magnitude < 1e-6f) {
            continue; // Skip if magnitude too small
        }
        sun_vec = sun_vec / magnitude;

        // Count inliers
        int inlier_count = 0;
        for (int i = 0; i < n_sensors; i++) {
            float3 normal = {normals[i][0], normals[i][1], normals[i][2]};
            float expected_signal = fmax(0.0f, dot(sun_vec, normal));
            float normalized_actual = signals[i] / magnitude; // Normalize actual signal

            float error = fabs(expected_signal - normalized_actual);
            if (error < INLIER_THRESHOLD) {
                inlier_count++;
            }
        }

        // Update best solution if this one has more inliers
        if (inlier_count > best_inlier_count) {
            best_inlier_count = inlier_count;
            best_sun_vector = sun_vec;
        }
    }

    return best_inlier_count >= 3; // Need at least 3 inliers for valid solution
}

