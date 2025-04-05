/**
 * @author Niklas Vainio
 * @version 0.1
 *
 * Generic C functions for multiplying arbitrarily large matrices
 */

#include "matrix_utils.h"
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
void matmul_square(float *A, float *B, float *out, int N)
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
void matmul(float *A, float *B, float *out, int M, int P, int N)
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

void mat_inverse()
{
}

static void test_transpose()
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

static void test_matmul()
{
    // Test matrix multiplication
    float A[15] = {
        1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 2, 4,
    };

    float B[12] = {
        2, 1, 0, 7, 3, 1, 0, 3, 0, 0, 4, -5,
    };

    float out[15];
    matmul(A, B, out, 5, 3, 4);

    printf("out: ");
    for (int i = 0; i < 15; i++)
    {
        printf("%f ", out[i]);
    }
    printf("\n");
}

void test_matrix_utils()
{
    // test_matmul();
    test_transpose();
}
