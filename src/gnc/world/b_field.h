#pragma once

#include "linalg.h"
#include "slate.h"

// compute_B computes the magnetic field
void compute_B(slate_t *slate);
// tests B field values
void test_compute_B(slate_t *slate);
// test_legendre_values
void test_legendre_polynomials();
