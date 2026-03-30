#pragma once

#include "constants.h"
#include "linalg.h"
#include "macros.h"
#include "pico/types.h"

#include "drivers/imu/imu.h"
#include "drivers/magnetometer/magnetometer.h"
#include "drivers/watchdog_motor/watchdog.h"

#include "apps/adcs_new/params.h"
#include "apps/adcs_new/states/states.h"
#include "apps/adcs_new/tasks/tasks.h"

typedef struct samwise_slate_rewrite {
    imu_data_t imu_data;
    magnetometer_data_t magnetometer_data;
    watchdog_t watchdog;
    StateMachine_t state_machine;
    TaskHandle_t task_handles[NTASKS];
} slate_t;

