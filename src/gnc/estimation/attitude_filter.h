/**
 * @author Lundeen Cahilly, Niklas Vainio, Chen Li
 * @brief This file implements an attitude EKF
 * using modified Rodrigues parameters (MRPs)
 * @date 2025-10-16
 *
 * Based on this paper:
 * https://ntrs.nasa.gov/api/citations/19960035754/downloads/19960035754.pdf
 * General EKF info from
 * https://stanfordasl.github.io/PoRA-I/aa174a_aut2526/resources/PoRA.pdf
 */

#pragma once

#include "linalg.h"
#include "slate.h"

using namespace linalg::aliases;

void attitude_filter_init(slate_t *slate);

void attitude_filter_propagate(slate_t *slate);

void attitude_filter_update(slate_t *slate, char sensor_type);

#ifdef TEST
void ekf_propagator_test(slate_t *slate);
void ekf_time_test(slate_t *slate);
void ekf_convergence_test(slate_t *slate);
void ekf_mrp_wrapping_test(slate_t *slate);
void ekf_high_rate_tumble_test(slate_t *slate);
void ekf_convergence_logging_test(slate_t *slate);
void ekf_stationary_bias_test(slate_t *slate);
void ekf_rotation_with_bias_test(slate_t *slate);
#endif