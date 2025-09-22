/**
 * @author Lundeen Cahilly
 * @date 2025-02-10
 *
 * This file defines a magnetic field model based on the IGRF-14 2025
 * coefficients. It computes the magnetic field vector in based on the
 * satellite's geodetic coordinates (altitude, latitude, longitude).
 */

#pragma once

#include "slate.h"

// compute_B computes the magnetic field
// returns true if successful, false if there was an error
bool compute_B(slate_t *slate);
