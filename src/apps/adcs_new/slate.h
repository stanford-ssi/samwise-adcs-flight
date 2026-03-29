#pragma once

#include "constants.h"
#include "linalg.h"
#include "macros.h"
#include "pico/types.h"

#include "drivers/imu/imu.h"
#include "drivers/magnetometer/magnetometer.h"

#include "apps/adcs_new/params.h"
#include "apps/adcs_new/states/states.h"

typedef struct samwise_slate_rewrite {
    IMU_data_t IMU_data;
    magnetometer_data_t MAG_data;
    StateMachine_t state_machine;
} slate_t;

