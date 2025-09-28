/**
 * @author Chen Li
 * @date 2025-09-05
 *
 * This file calculates Modified Julian Date based on GPS time, difference
 * between UTC and UT1 is ignored
 */

#pragma once

#include "slate.h"

void compute_MJD(slate_t *slate);