// Magnetometer Calibration Parameters
// Generated on 20250821_200220
// Method: ellipsoid

// Hard iron offset correction (sensor units)
constexpr float3 MAG_HARD_IRON_OFFSET = float3{0.062558, 0.200758, 0.042450};

// Soft iron matrix correction (in sensor units)
constexpr float3x3 MAG_SOFT_IRON_MATRIX = {
    {1.016360f, -0.007422f, -0.011832f},
    {-0.007422f, 0.950359f, -0.011544f},
    {-0.011832f, -0.011544f, 1.019076f}
};

