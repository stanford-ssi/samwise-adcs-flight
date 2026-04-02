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
#include "apps/adcs_app/slate.h"
#include "drivers/gps/gps.h"

typedef struct b_field {
    float3 b_eci;
    float3 b_ecef;
    
    float3 b_rpt;
    float3 b_enu;
} b_field_t;

bool compute_B(const gps_data_processed_t &gps_data, b_field_t &b_field);

#ifdef TEST
void test_b_field_reference_points(gps_data_processed_t &gps_data, 
        b_field_t &b_field);
void test_b_field_mapping(gps_data_processed_t &gps_data, 
        b_field_t &b_field);
void test_b_field_ecef_conversion(gps_data_processed_t &gps_data, 
        b_field_t &b_field);
#endif
