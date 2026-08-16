/**
 * @author Lundeen Cahilly
 * @date 2025-02-10
 *
 * This file defines a magnetic field model based on the IGRF-14 2025
 * coefficients. It computes the magnetic field vector in based on the
 * satellite's geodetic coordinates (altitude, latitude, longitude).
 */

#pragma once

#include "macros.h"
#include "drivers/gps/gps.h"

struct b_field_t {
    // True when the vectors below came from a successful compute_B() call.
    // compute_B() returns early without touching them on bad geodetic input,
    // so callers must not treat a stale reference as current.
    bool valid;

    float3 b_eci;
    float3 b_ecef;

    float3 b_rpt;
    float3 b_enu;
};

bool compute_B(const gps_data_processed_t &gps_data, b_field_t &b_field);

#ifdef TEST
void test_b_field_reference_points(gps_data_processed_t &gps_data, 
        b_field_t &b_field);
void test_b_field_mapping(gps_data_processed_t &gps_data, 
        b_field_t &b_field);
void test_b_field_ecef_conversion(gps_data_processed_t &gps_data, 
        b_field_t &b_field);
#endif
