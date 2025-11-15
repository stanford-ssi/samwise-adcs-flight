/**
 * @author Lundeen Cahilly
 * @date 2025-11-08
 *
 * This task is responsible for allocating the magnetorquer dipole moments
 * given the requested control moment
 */

#pragma once 

#include "slate.h"
#include "linalg.h"
#include "macros.h"
using namespace linalg::aliases;

void allocate_magnetorquers(slate_t *slate);

#ifdef TEST
void test_magtorq_allocation(slate_t *slate);
#endif