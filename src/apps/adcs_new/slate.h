#pragma once
#include "FreeRTOS.h"
#include "semphr.h"

#include "constants.h"
#include "linalg.h"
#include "macros.h"
#include "pico/types.h"

#include "drivers/gps/gps.h"
#include "drivers/imu/imu.h"
#include "drivers/power_monitor/power_monitor.h"
#include "drivers/magnetometer/magnetometer.h"
#include "drivers/magnetorquers/magnetorquers.h"
#include "drivers/watchdog_motor/watchdog.h"
#include "drivers/sun_sensors/sun_sensors.h"

#include "gnc/mekf/filter.h"
#include "gnc/world/sun_vector.h"
#include "gnc/world/b_field.h"
#include "gnc/control/bdot.h"

#include "apps/adcs_new/params.h"
#include "apps/adcs_new/states/states.h"
#include "apps/adcs_new/tasks/tasks.h"


typedef struct samwise_slate_rewrite {
    // ACTUATORS
    watchdog_t watchdog;

    // SENSORS
    imu_data_t imu_data;
    MagnetometerData magnetometer_data;
    gps_data_processed_t gps_data;
    sun_sensor_data_t sun_sensor;
    power_monitor_t power_monitor;

    // DATA
    sun_vector_t sun_vector;
    b_field_t b_field;

    // MEKF
    SensorFusion magnetometer_fusion = SensorFusion(0.01f);
    SensorFusion sun_vector_fusion = SensorFusion(0.01f);
    AttitudeFilter attitude_filter = AttitudeFilter(I_BODY,
            0.01f, IMU_GYRO_VARIANCE, 0.05f);

    // CONTROL
    SemaphoreHandle_t mag_mutex;
    BDotController bdot;
    Magnetorquer magtorq;

    // UTIL
    StateMachine_t state_machine;
    TaskHandle_t task_handles[NTASKS];
} slate_t;

