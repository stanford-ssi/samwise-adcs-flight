#include "apps/adcs_new/slate.h"
#include "apps/adcs_new/pins.h"
#include "apps/adcs_new/tasks/tasks.h"
#include "apps/adcs_new/states/states.h"

#include "drivers/imu/imu.h"
#include "drivers/magnetometer/magnetometer.h"
#include "drivers/watchdog_motor/watchdog.h"
#include "drivers/sun_sensors/sun_sensors.h"
#include "drivers/sun_sensors/rp2350b_adc.h"
#include "drivers/sun_sensors/ads7830.h"

extern slate_t slate;

static StaticTask_t watchdog_tcb;
static StackType_t  watchdog_stack[256];

static StaticTask_t state_machine_tcb;
static StackType_t state_machine_stack[256];

static StaticTask_t gps_tcb;
static StackType_t  gps_stack[256];

static StaticTask_t magnetometer_tcb;
static StackType_t  magnetometer_stack[256];

static StaticTask_t imu_tcb;
static StackType_t imu_stack[256];

static StaticTask_t sun_sensor_tcb;
static StackType_t sun_sensor_stack[256];

static StaticTask_t sun_sensor_fusion_tcb;
static StackType_t sun_sensor_fusion_stack[256];

static StaticTask_t magnetometer_fusion_tcb;
static StackType_t magnetometer_fusion_stack[256];

static StaticTask_t attitude_propagate_tcb;
static StackType_t attitude_propagate_stack[256];

static StaticTask_t reset_estimate_tcb;
static StackType_t reset_estimate_stack[256];

static void init_i2c_buses() {
    // Initialize I2c pins and buses
    gpio_init(SAMWISE_ADCS_I2C0_SCL);
    gpio_init(SAMWISE_ADCS_I2C0_SDA);
    gpio_set_function(SAMWISE_ADCS_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_set_function(SAMWISE_ADCS_I2C0_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SAMWISE_ADCS_I2C0_SCL);
    gpio_pull_up(SAMWISE_ADCS_I2C0_SDA);

    gpio_init(SAMWISE_ADCS_I2C1_SCL);
    gpio_init(SAMWISE_ADCS_I2C1_SDA);
    gpio_set_function(SAMWISE_ADCS_I2C1_SCL, GPIO_FUNC_I2C);
    gpio_set_function(SAMWISE_ADCS_I2C1_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SAMWISE_ADCS_I2C1_SCL);
    gpio_pull_up(SAMWISE_ADCS_I2C1_SDA);

    i2c_init(i2c0, SAMWISE_ADCS_I2C0_BAUD);
    i2c_init(i2c1, SAMWISE_ADCS_I2C1_BAUD);
}

void init_main() {
    // ==============================================================
    // INIT STATE MACHINE
    // ==============================================================
    init_state_machine();
    slate.state_machine.state_machine_handler = xTaskCreateStatic(
        vTaskStateMachine,   // function
        "state machine",      // name
        256,          // stack depth
        nullptr,      // params
        2,            // priority
        state_machine_stack,  // stack buffer
        &state_machine_tcb    // TCB buffer
    );

    // ==============================================================
    // INIT I2C
    // ==============================================================
    init_i2c_buses();

    // ==============================================================
    // WATCHDOG INIT 
    // ==============================================================
    slate.watchdog = watchdog_mk(SAMWISE_ADCS_WATCHDOG_FEED);
    watchdog_init(&slate.watchdog);
    slate.task_handles[TASK_WATCHDOG] = xTaskCreateStatic(
        vTaskWatchdog,   // function
        "watchdog task",      // name
        256,          // stack depth
        nullptr,      // params
        2,            // priority
        watchdog_stack,  // stack buffer
        &watchdog_tcb    // TCB buffer
    );

    // ==============================================================
    // IMU INIT 
    // ==============================================================
    LOG_INFO("[sensor] Initializing IMU...");
    bool imu_result = imu_init();
    slate.imu_data.imu_alive = imu_result;

    if (!imu_result)
    {
        LOG_ERROR("[sensor] Error initializing IMU - deactivating!");
    }
    LOG_INFO("Hello?");

    slate.imu_data.imu_data_valid = false;

    LOG_INFO("[sensor] IMU Initialization Complete! IMU alive: %s",
             slate.imu_data.imu_alive ? "true" : "false");

    slate.task_handles[TASK_IMU] = xTaskCreateStatic(
        vTaskIMU,   // function
        "imu sensor task",      // name
        256,          // stack depth
        nullptr,      // params
        2,            // priority
        imu_stack,  // stack buffer
        &imu_tcb    // TCB buffer
    );

    // ==============================================================
    // MAGNETOMETER INIT 
    // ==============================================================
    LOG_INFO("[sensor] Initializing magnetometer...");
    rm3100_error_t magnetometer_result = rm3100_init();
    slate.magnetometer_data.magnetometer_alive 
        = (magnetometer_result == RM3100_OK);

    if (magnetometer_result != RM3100_OK)
    {
        LOG_ERROR("[sensor] Error initializing magnetometer - deactivating!");
    }

    slate.magnetometer_data.magnetometer_data_valid = false;

    LOG_INFO(
        "[sensor] Magnetometer Initialization Complete! Magnetometer alive: %s",
        slate.magnetometer_data.magnetometer_alive ? "true" : "false");

    slate.task_handles[TASK_MAGNETOMETER] = xTaskCreateStatic(
        vTaskMagnetometer,   // function
        "magnetometer sensor task",      // name
        256,          // stack depth
        nullptr,      // params
        2,            // priority
        magnetometer_stack,  // stack buffer
        &magnetometer_tcb    // TCB buffer
    );
    // ==============================================================
    // GPS INIT 
    // ==============================================================
    LOG_INFO("[sensor] Initializing GPS...");

    bool gps_result = gps_init();
    slate.gps_data.gps_alive = gps_result;

    if (!gps_result)
    {
        LOG_ERROR("[sensor] Error initializing GPS - deactivating!");
    }
    slate.gps_data.gps_data_valid = false;

    LOG_INFO("[sensor] GPS Initialization Complete! GPS alive: %s",
             slate.gps_data.gps_alive ? "true" : "false");

    slate.task_handles[TASK_GPS] = xTaskCreateStatic(
        vTaskGPS,   // function
        "gps task",      // name
        256,          // stack depth
        nullptr,      // params
        2,            // priority
        gps_stack,  // stack buffer
        &gps_tcb    // TCB buffer
    );
    // ==============================================================
    // SUN SENSOR INIT 
    // ==============================================================
    LOG_INFO("[sensor] Initializing sun sensors rp2350b adc...");
    bool rp2350b_adc_result = rp2350b_adc_init();
    if (!rp2350b_adc_result)
    {
        LOG_ERROR("[sensor] Error initializing rp2350b_adc - deactivating!");
    }

    LOG_INFO("[sensor] Initializing sun sensors ads7830...");
    bool ads7830_result = ads7830_init();
    if (!ads7830_result)
    {
        LOG_ERROR("[sensor] Error initializing ads7830 - deactivating!");
    }

    // Write alive status for each sun sensor
    for (uint8_t i = 0; i < NUM_SUN_SENSORS; i++)
    {
        if (i < NUM_SUN_SENSORS / 2)
        {
            // First 8 sensors are from rp2350b adc
            slate.sun_sensor.sun_sensor_alive[i] = rp2350b_adc_result;
        }
        else
        {
            // Last 8 sensors are from ads7830
            slate.sun_sensor.sun_sensor_alive[i] = ads7830_result;
        }
        slate.sun_sensor.data_valid[i] = false;
    }

    slate.sun_sensor.sun_vector_valid = false;

    LOG_INFO("[sensor] Sun sensors alive status:");
    for (uint8_t i = 0; i < NUM_SUN_SENSORS; i++)
    {
        LOG_INFO("  Sensor %2d: %s", i,
                 slate.sun_sensor.sun_sensor_alive[i] ? "true" : "false");
    }

    slate.task_handles[TASK_SUN_SENSOR] = xTaskCreateStatic(
        vTaskSunSensor,   // function
        "sun sensor task",      // name
        256,          // stack depth
        nullptr,      // params
        2,            // priority
        sun_sensor_stack,  // stack buffer
        &sun_sensor_tcb    // TCB buffer
    );

    // ==============================================================
    // INIT SENSOR FUSION 
    // ==============================================================

    slate.task_handles[TASK_SUN_VECTOR_FUSION] = xTaskCreateStatic(
        vTaskSunVectorFusion,   // function
        "sun sensor fusion task",      // name
        256,          // stack depth
        nullptr,      // params
        2,            // priority
        sun_sensor_fusion_stack,  // stack buffer
        &sun_sensor_fusion_tcb    // TCB buffer
    );

    slate.task_handles[TASK_MAGNETOMETER_FUSION] = xTaskCreateStatic(
        vTaskMagnetometerFusion,   // function
        "magnetometer fusion task",      // name
        256,          // stack depth
        nullptr,      // params
        2,            // priority
        magnetometer_fusion_stack,  // stack buffer
        &magnetometer_fusion_tcb    // TCB buffer
    );

    slate.task_handles[TASK_PROPAGATE] = xTaskCreateStatic(
        vTaskPropagate,   // function
        "attitude propagation task",      // name
        256,          // stack depth
        nullptr,      // params
        2,            // priority
        attitude_propagate_stack,  // stack buffer
        &attitude_propagate_tcb    // TCB buffer
    );


}
