/**
 * @author Niklas Vainio
 * @version 0.1
 *
 * Generic C functions for multiplying arbitrarily large matrices
 */

void mat_mul_square(const float *A, const float *B, float *out, int N);

void mat_mul(const float *A, const float *B, float *out, int M, int P, int N);

void mat_add(const float *A, const float *B, float *out, int M, int N);

void mat_sub(const float *A, const float *B, float *out, int M, int N);

void mat_transpose(const float *A, float *out, int M, int N);

void mat_inverse(const float *A, float *out, int N);

float mat_log_frobenius(const float *A, int N);

bool mat_contains_nan(const float *A, int M, int N);

void mat_pseudoinverse(const float *A, float *out, int M, int N);

void test_matrix_utils();