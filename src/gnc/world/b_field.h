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
#include "slate.h"

bool compute_B(slate_t *slate);

#ifdef TEST
void test_b_field_reference_points(slate_t *slate);
void test_b_field_mapping(slate_t *slate);
void test_b_field_ecef_conversion(slate_t *slate);
#endif