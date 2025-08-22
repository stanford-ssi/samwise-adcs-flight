// Magnetometer Calibration Parameters
// Generated on 20250821_200829
// Method: ellipsoid

// Hard iron offset correction (sensor units)
constexpr float3 MAG_HARD_IRON_OFFSET = float3{5.292912, -8.688055, 3.885803};

// Soft iron matrix correction (in sensor units)
constexpr float3x3 MAG_SOFT_IRON_MATRIX = {
    {0.020153f, 0.000078f, 0.000100f},
    {0.000078f, 0.022109f, 0.001336f},
    {0.000100f, 0.001336f, 0.020490f}
};

