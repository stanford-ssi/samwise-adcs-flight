/**
 * @author Niklas Vainio
 * @version 0.1
 *
 * Generic C functions for multiplying arbitrarily large matrices
 */

void matmul_square(float *A, float *B, float *out, int N);

void matmul(float *A, float *B, float *out, int M, int N, int P);

void mat_transpose(float *A, float *out, int M, int N);

void test_matrix_utils();