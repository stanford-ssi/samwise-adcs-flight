/**
 * @author  Niklas Vainio
 * @date    2025-05-29
 *
 * Task to run tests - feel free to hook new code in here
 */

#include "test_task.h"
#include "hardware/i2c.h"
#include "macros.h"

#include "../tests/adm1176_test.h"
#include "../tests/i2c_scanner.h"
#include "../tests/magnetorquer_tests.h"
#include "telemetry_task.h"

void test_task_init(slate_t *slate)
{
    LOG_INFO("[test] Initializing test task!");
    // telemetry_task_init(slate);
    init_power_monitor();
    // magnetorquer_tests_init();
}

void test_task_dispatch(slate_t *slate)
{
    LOG_INFO("[test] TEST TASK IS DISPATCHING");
    // telemetry_task_dispatch(slate);
    read_power_monitor();
    // scan_i2c_bus(i2c1, "I2C1");
    // scan_i2c_bus(i2c0, "I2C0");
    // magnetorquer_tests_dispatch();
}

sched_task_t test_task = {.name = "test",
                          .dispatch_period_ms = 10,
                          .task_init = &test_task_init,
                          .task_dispatch = &test_task_dispatch,

                          /* Set to an actual value on init */
                          .next_dispatch = 0};
