# Sun Vector Calculation

Calculates theoretical Sun position in ECI coordinates using GPS time and orbital mechanics.

**Location:** `src/gnc/sun_vector.{h,cpp}`

## Key Constants
- `DEG2RAD = 3.14159f / 180.0f`

## Functions
```cpp
void compute_sun_vector_eci(slate_t *slate);  // Compute sun vector from MJD
void test_sun_vector_eci(slate_t *slate);     // Test function
```

## Implementation
**Input:** `slate->MJD` (Modified Julian Date from GPS)
**Output:** `slate->sun_vector_eci` (unit vector to Sun in ECI frame)

**Algorithm:**
```cpp
JD = slate->MJD + 2400000.5f;                    // Convert to Julian Date
T = (JD - 2451545.0f) / 36525.0f;              // Julian centuries since J2000
L0 = 280.46646f + 36000.76983f * T + 0.0003032f * T * T;  // Mean longitude
M = 357.52911f + 35999.05029f * T - 0.0001537f * T * T;   // Mean anomaly
// ... equation of center calculation
lambda = L0 + C;                                 // True longitude
epsilon = 23.439291f - 0.0130042f * T;          // Obliquity
X = cos(lambda_rad);                             // ECI coordinates
Y = cos(epsilon_rad) * sin(lambda_rad);
Z = sin(epsilon_rad) * sin(lambda_rad);
```

Used for attitude reference by comparing theoretical vs measured sun direction.