/**
 * @author Niklas Vainio
 * @brief This file implements the main attitude EFK
 * @date 2025-05-03
 */

#pragma once

#include "slate.h"

void attitude_filter_init(slate_t *slate);

void attitude_filter_propagate(slate_t *slate, float dt);

void attitude_filter_update(slate_t *slate, quaternion q_meas_eci_to_principal);

void test_attitude_filter(slate_t *slate);