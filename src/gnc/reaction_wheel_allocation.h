#pragma once

#include "constants.h"
#include "linalg.h"
#include "slate.h"

// Function declarations
linalg::vec<float, 3> compute_control_torque(
    linalg::vec<float, 4> q_current, linalg::vec<float, 4> q_desired,
    linalg::vec<float, 3> omega_current, linalg::vec<float, 3> omega_desired);

linalg::vec<float, 4> allocate_reaction_wheels(
    linalg::vec<float, 4> q_current, linalg::vec<float, 4> q_desired,
    linalg::vec<float, 3> omega_current, linalg::vec<float, 3> omega_desired);

void test_reaction_wheel_allocation();