#include "apps/adcs_new/slate.h"
#include "apps/adcs_new/pins.h"
#include "apps/adcs_new/tasks/tasks.h"
#include "apps/adcs_new/states/states.h"

#include "drivers/imu/imu.h"
#include "drivers/magnetometer/magnetometer.h"
#include "drivers/watchdog_motor/watchdog.h"

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

    // Initialize state machine
    // slate.magnetometer_data.last_mag_read_start = get_absolute_time();

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
    slate.task_handles[TASK_SUN_SENSOR] = xTaskCreateStatic(
        vTaskSunSensor,   // function
        "sun sensor task",      // name
        256,          // stack depth
        nullptr,      // params
        2,            // priority
        sun_sensor_stack,  // stack buffer
        &sun_sensor_tcb    // TCB buffer
    );
}
