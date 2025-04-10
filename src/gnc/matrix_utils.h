/**
 * @author Niklas Vainio
 * @version 0.1
 *
 * Generic C functions for multiplying arbitrarily large matrices
 */

void mat_mul_square(float *A, float *B, float *out, int N);

void mat_mul(float *A, float *B, float *out, int M, int N, int P);

void mat_transpose(float *A, float *out, int M, int N);

void mat_inverse(float *A, float *out, int N);

void test_matrix_utils();