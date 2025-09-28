/**
 * @author  Chen Li
 * @date    2025-09-08
 *
 * Extended Kalman Filter test
 */

#pragma once

#include "constants.h"
#include "linalg.h"
#include "slate.h"

void ekf_test(slate_t *slate);
void expected_quaternion(slate_t *slate, quaternion &q_expected);
void attitude_filter_software_test(slate_t *slate);