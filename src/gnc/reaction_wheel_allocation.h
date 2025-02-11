#pragma once

#include "constants.h"
#include "linalg.h"
#include "slate.h"

linalg::vec<float, 3> compute_control_torque(linalg::vec<float, 4> x_current,
                                             linalg::vec<float, 4> x_desired);
linalg::vec<float, 4> allocate_reaction_wheels(linalg::vec<float, 4> x_current,
                                               linalg::vec<float, 4> x_desired);
