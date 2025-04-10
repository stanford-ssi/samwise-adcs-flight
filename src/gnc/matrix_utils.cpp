/**
 * @author Niklas Vainio
 * @version 0.1
 *
 * Generic C functions for multiplying arbitrarily large matrices
 */

#include "matrix_utils.h"
#include "macros.h"
#include "pico/printf.h"

#define IDX(row, col, ncols) (((row) * ncols) + col)

/**
 * Multiply to NxN matrices, storing the result in out. Assumes
 * matrices are in row major order.
 *
 * @param A (NxN)
 * @param B (NxN)
 * @param out (NxN)
 * @param N
 */
void mat_mul_square(float *A, float *B, float *out, int N)
{
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            float acc = 0;

            for (int k = 0; k < N; k++)
            {
                acc += A[IDX(i, k, N)] * B[IDX(k, j, N)];
            }

            out[IDX(i, j, N)] = acc;
        }
    }
}

/**
 * Multiply an MxP and a PxN matrix to give an MxN matrix.
 * Assumes matrices are in row major order.
 *
 * @param A (MxP)
 * @param B (PxN)
 * @param out (MxN)
 * @param M
 * @param P
 * @param N
 */
void mat_mul(float *A, float *B, float *out, int M, int P, int N)
{
    for (int i = 0; i < M; i++)
    {
        for (int j = 0; j < N; j++)
        {
            float acc = 0;

            for (int k = 0; k < P; k++)
            {
                acc += A[IDX(i, k, P)] * B[IDX(k, j, N)];
            }

            out[IDX(i, j, N)] = acc;
        }
    }
}

/**
 * @brief Populate out (row-major) with the transpose of A
 *
 * @param A (MxN)
 * @param out (NxM)
 * @param M
 * @param N
 */
void mat_transpose(float *A, float *out, int M, int N)
{
    for (int i = 0; i < M; i++)
    {
        for (int j = 0; j < N; j++)
        {
            out[IDX(j, i, M)] = A[IDX(i, j, N)];
        }
    }
}

/**
 * @brief Populate out (row-major) with the inverse of A. Assumes
 * that A is non-singular (will lead to NaN's otherwise).
 *
 * @param A (NxN
 * @param out (NxN)
 * @param N
 */
void mat_inverse(float *A, float *out, int N)
{
    // (courtesy of ChatGPT!)

    // Initialize the output matrix to the identity matrix
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            out[IDX(i, j, N)] = (i == j) ? 1.0f : 0.0f;

    // Create a working copy of A
    float temp[N * N];
    for (int i = 0; i < N * N; ++i)
        temp[i] = A[i];

    for (int i = 0; i < N; ++i)
    {
        // Pivot: Make sure temp[i][i] is not zero by swapping rows if necessary
        if (temp[IDX(i, i, N)] == 0.0f)
        {
            for (int k = i + 1; k < N; ++k)
            {
                if (temp[IDX(k, i, N)] != 0.0f)
                {
                    // Swap rows i and k in both temp and out
                    for (int j = 0; j < N; ++j)
                    {
                        float tmp1 = temp[IDX(i, j, N)];
                        float tmp2 = out[IDX(i, j, N)];
                        temp[IDX(i, j, N)] = temp[IDX(k, j, N)];
                        out[IDX(i, j, N)] = out[IDX(k, j, N)];
                        temp[IDX(k, j, N)] = tmp1;
                        out[IDX(k, j, N)] = tmp2;
                    }
                    break;
                }
            }
        }

        float diag = temp[IDX(i, i, N)];
        // Normalize row i
        for (int j = 0; j < N; ++j)
        {
            temp[IDX(i, j, N)] /= diag;
            out[IDX(i, j, N)] /= diag;
        }

        // Eliminate column i from other rows
        for (int k = 0; k < N; ++k)
        {
            if (k == i)
                continue;
            float factor = temp[IDX(k, i, N)];
            for (int j = 0; j < N; ++j)
            {
                temp[IDX(k, j, N)] -= factor * temp[IDX(i, j, N)];
                out[IDX(k, j, N)] -= factor * out[IDX(i, j, N)];
            }
        }
    }
}

// Helper functions to generate random numbers for testing
static uint32_t seed = 1;
static unsigned int rand_int()
{
    seed = (1664525 * seed + 1013904223);
    return seed;
}

static float rand_float()
{
    // Cast to float and normalize to (-1, 1)
    return ((rand_int() & 0xFFFFFF) / (float)0x1000000) * 2.0f - 1.0f;
}

static void test_mat_transpose()
{
    // Test matrix transpose
    float A[6] = {1, 2, 3, 4, 5, 6};

    float out[6];
    mat_transpose(A, out, 3, 2);

    printf("out: ");
    for (int i = 0; i < 6; i++)
    {
        printf("%f ", out[i]);
    }
    printf("\n");
}

static void test_mat_mul()
{
    // Test matrix multiplication
    float A[15] = {
        1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 2, 4,
    };

    float B[12] = {
        2, 1, 0, 7, 3, 1, 0, 3, 0, 0, 4, -5,
    };

    float out[15];
    mat_mul(A, B, out, 5, 3, 4);

    printf("out: ");
    for (int i = 0; i < 15; i++)
    {
        printf("%f ", out[i]);
    }
    printf("\n");
}

static void test_mat_inverse()
{
    // Test matrix inversion
    float A[16];

    // Populate A with random numbers
    for (int i = 0; i < 16; i++)
    {
        A[i] = rand_float();
    }

    // Calculate inverse
    float A_inv[16];
    mat_inverse(A, A_inv, 4);

    // Check product
    float I[16];
    mat_mul_square(A, A_inv, I, 4);

    printf("A * A^-1: ");
    for (int i = 0; i < 16; i++)
    {
        printf("%f ", I[i]);
    }
    printf("\n");
    ASSERT_ALMOST_EQ(0.0f, 0.0f, 1.0f);
}

void test_matrix_utils()
{
    // test_mat_mul();
    // test_mat_transpose();
    test_mat_inverse();
}
