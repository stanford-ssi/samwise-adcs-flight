// Magnetometer Calibration Parameters
// Generated on 20250810_213022
// Method: ellipsoid

#ifndef MAG_CALIBRATION_H
#define MAG_CALIBRATION_H

// Hard iron offset
static const float mag_offset[3] = {
    -10.879795f,  // X offset
    4.468930f,  // Y offset
    -1.344230f   // Z offset
};

// Soft iron correction matrix
static const float mag_transform[3][3] = {
    {0.024522f, -0.001219f, -0.000468f},
    {-0.001219f, 0.027507f, 0.000960f},
    {-0.000468f, 0.000960f, 0.028177f}
};

// Full calibration function
static inline void apply_mag_calibration(float raw[3], float cal[3]) {
    float centered[3] = {
        raw[0] - mag_offset[0],
        raw[1] - mag_offset[1],
        raw[2] - mag_offset[2]
    };

    cal[0] = mag_transform[0][0] * centered[0] + mag_transform[0][1] * centered[1] + mag_transform[0][2] * centered[2];
    cal[1] = mag_transform[1][0] * centered[0] + mag_transform[1][1] * centered[1] + mag_transform[1][2] * centered[2];
    cal[2] = mag_transform[2][0] * centered[0] + mag_transform[2][1] * centered[1] + mag_transform[2][2] * centered[2];
}

#endif // MAG_CALIBRATION_H
