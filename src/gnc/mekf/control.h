/*
 * Carson Lauer control utilities. 
 *
 * It features a controller class that is used to calculate control
 * vectors as well as save state about target goal and heading.
 */

#pragma once
#include "linalg.h"

using namespace linalg;
using namespace linalg::aliases;

enum class ControlMode {
    safe = 0,
    nadir,
    bdot,
    stanford
};

class SatelliteController {
    
    float3 heading_eci_; // could be variable
    float3 heading_body_; // variable
    float3 sight_body_; // constant
    public:
    float3 torque_;
};

