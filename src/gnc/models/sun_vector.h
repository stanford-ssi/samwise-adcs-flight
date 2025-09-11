/**
 * @author Chen Li and Lundeen Cahilly
 * @date 2025-07-04
 */

#pragma once

#include "constants.h"
#include "linalg.h"
#include "slate.h"

void compute_sun_vector_eci(slate_t *slate);
void test_sun_vector_eci(slate_t *slate);
void test_sun_vector_year(slate_t *slate);