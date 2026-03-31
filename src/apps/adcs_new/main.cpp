#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rtos_setup.h"

#include "apps/adcs_new/tasks/tasks.h"
#include "apps/adcs_new/states/states.h"
#include "apps/adcs_new/slate.h"
#include "apps/adcs_new/init.h"

slate_t slate;

int main() {
    stdio_init_all();


    // adcs_tasks_init();
    sleep_ms(5000);
    LOG_INFO("Starting Task Machine");

    init_main();

    sleep_ms(3000);

    LOG_INFO("Init done, starting state machine");

    vTaskStartScheduler();
}
