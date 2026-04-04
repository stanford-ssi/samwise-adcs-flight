#pragma once

#include "constants.h"
#include "linalg.h"
#include "macros.h"
#include "pico/types.h"

#include "drivers/gps/gps.h"
#include "drivers/imu/imu.h"
#include "drivers/magnetometer/magnetometer.h"
#include "drivers/watchdog_motor/watchdog.h"
#include "drivers/sun_sensors/sun_sensors.h"

#include "gnc/mekf/filter.h"
#include "gnc/world/sun_vector.h"
#include "gnc/world/b_field.h"

#include "apps/adcs_new/params.h"
#include "apps/adcs_new/states/states.h"
#include "apps/adcs_new/tasks/tasks.h"


typedef struct samwise_slate_rewrite {
    // ACTUATORS
    watchdog_t watchdog;

    // SENSORS
    imu_data_t imu_data;
    magnetometer_data_t magnetometer_data;
    gps_data_processed_t gps_data;
    sun_sensor_data_t sun_sensor;

    // DATA
    sun_vector_t sun_vector;
    b_field_t b_field;

    // MEKF
    SensorFusion magnetometer_fusion = SensorFusion(0.01);
    SensorFusion sun_vector_fusion = SensorFusion(0.01);
    AttitudeFilter attitude_filter = AttitudeFilter(I_BODY,
            0.001f, IMU_GYRO_VARIANCE, 0.05f);

    // UTIL
    StateMachine_t state_machine;
    TaskHandle_t task_handles[NTASKS];
} slate_t;

