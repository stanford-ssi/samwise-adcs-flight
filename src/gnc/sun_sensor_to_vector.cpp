/**
 * @author Chen Li and Lundeen Cahilly
 * @date 2025-08-24
 */

#include "gnc/sun_sensor_to_vector.h"
#include "constants.h"
#include "gnc/matrix_utils.h"
#include "macros.h"
#include "pico/stdlib.h"

// Global storage for computed pseudoinverse (3x8)
static float sensor_pseudoinverse[3][8];
static bool pseudoinverse_computed = false;

// --- TODO: add yz photodiodes ---

// Define actual sun sensor normal vectors (8x3 matrix)
// TODO: update with final structure geometry
const float sensor_normals[8][3] = {
    {1, 0, 1},   // sun_pyramid_1_1 [0]
    {0, -1, 1},  // sun_pyramid_1_2 [1]
    {-1, 0, 1},  // sun_pyramid_1_3 [2]
    {0, 1, 1},   // sun_pyramid_1_4 [3]
    {-1, 0, -1}, // sun_pyramid_2_1 [4]
    {0, -1, -1}, // sun_pyramid_2_2 [5]
    {1, 0, -1},  // sun_pyramid_2_3 [6]
    {0, 1, -1},  // sun_pyramid_2_4 [7]
};

void compute_sensor_pseudoinverse();

void sun_sensors_to_attitude(slate_t *slate)
{
    // Ensure pseudoinverse is computed
    compute_sensor_pseudoinverse();

    // Use all 8 sensor readings directly (no differential approach)
    float sensor_readings[8];
    for (int i = 0; i < 8; i++)
    {
        sensor_readings[i] =
            static_cast<float>(slate->sun_sensors_intensities[i]);
    }

    // Find max intensity for threshold checking
    float I_max = sensor_readings[0];
    for (int i = 1; i < 8; i++)
    {
        if (sensor_readings[i] > I_max)
        {
            I_max = sensor_readings[i];
        }
    }

    // Check if we have enough light to make a measurement
    if (I_max < 1e-6f)
    {
        // No sun detected - set sun vector to zero
        slate->sun_vector_body.x = 0.0f;
        slate->sun_vector_body.y = 0.0f;
        slate->sun_vector_body.z = 0.0f;
        return;
    }

    // Compute sun vector using pseudoinverse: sun_vector = A+ * sensor_readings
    float sun_vector_raw[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            sun_vector_raw[i] +=
                sensor_pseudoinverse[i][j] * sensor_readings[j];
        }
    }

    // Normalize to unit vector
    float magnitude = sqrtf(sun_vector_raw[0] * sun_vector_raw[0] +
                            sun_vector_raw[1] * sun_vector_raw[1] +
                            sun_vector_raw[2] * sun_vector_raw[2]);

    if (magnitude > 1e-6f)
    {
        slate->sun_vector_body.x = sun_vector_raw[0] / magnitude;
        slate->sun_vector_body.y = sun_vector_raw[1] / magnitude;
        slate->sun_vector_body.z = sun_vector_raw[2] / magnitude;
    }
    else
    {
        // Degenerate case - set to zero
        slate->sun_vector_body.x = 0.0f;
        slate->sun_vector_body.y = 0.0f;
        slate->sun_vector_body.z = 0.0f;
    }
}

// Compute pseudoinverse: A+ = (A^T * A)^-1 * A^T
void compute_sensor_pseudoinverse()
{
    if (pseudoinverse_computed)
        return;

    // Step 1: Transpose A (8x3 -> 3x8)
    float A_transpose[3][8];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            A_transpose[i][j] = sensor_normals[j][i];
        }
    }

    // Step 2: Compute A^T * A (3x3 matrix)
    float AtA[3][3] = {0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 8; k++)
            {
                AtA[i][j] += A_transpose[i][k] * A_transpose[j][k];
            }
        }
    }

    // Step 3: Invert A^T * A using existing matrix_utils function
    float AtA_inv[3][3];
    mat_inverse(&AtA[0][0], &AtA_inv[0][0], 3);

    // Step 4: Compute pseudoinverse = (A^T * A)^-1 * A^T
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            sensor_pseudoinverse[i][j] = 0;
            for (int k = 0; k < 3; k++)
            {
                sensor_pseudoinverse[i][j] += AtA_inv[i][k] * A_transpose[k][j];
            }
        }
    }

    pseudoinverse_computed = true;
    LOG_INFO("Sun sensor pseudoinverse computed successfully");
}
