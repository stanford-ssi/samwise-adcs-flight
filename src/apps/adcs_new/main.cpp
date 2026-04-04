#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rtos_setup.h"

#include "init.h"

#include "apps/adcs_new/tasks/tasks.h"
#include "apps/adcs_new/states/states.h"
#include "apps/adcs_new/slate.h"

slate_t slate;

int main() {
    stdio_init_all();


    // adcs_tasks_init();
    sleep_ms(2000);
    LOG_INFO("[main] Starting Task Machine");

    init_tasks();

    vTaskStartScheduler();

}
