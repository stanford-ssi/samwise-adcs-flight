# Matrix Utilities

Generic C functions for matrix operations used in attitude filtering and GNC calculations.

**Location:** `src/gnc/matrix_utils.{h,cpp}`

## Functions
```cpp
void mat_mul_square(const float *A, const float *B, float *out, int N);      // Square matrix multiply
void mat_mul(const float *A, const float *B, float *out, int M, int N, int P); // General matrix multiply  
void mat_add(const float *A, const float *B, float *out, int M, int N);     // Matrix addition
void mat_transpose(const float *A, float *out, int M, int N);               // Matrix transpose
void mat_inverse(const float *A, float *out, int N);                        // Matrix inverse
float mat_frobenius(const float *A, float N);                               // Frobenius norm
void test_matrix_utils();                                                    // Test function
```

## Implementation
- **Storage:** Row-major order (C-style arrays)
- **In-place:** Some operations support in-place computation
- **Performance:** Optimized for small matrices used in attitude filtering

Used by attitude filter for covariance propagation, Kalman gain computation, and state updates.