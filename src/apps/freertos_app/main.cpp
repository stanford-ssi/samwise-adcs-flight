#include <cstdio>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rtos_setup.h"

void blink_task(void *) {
    constexpr uint LED = PICO_DEFAULT_LED_PIN;
    constexpr TickType_t delay = pdMS_TO_TICKS(500);

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    while (true) {
        gpio_put(LED, 1);
        vTaskDelay(delay);
        gpio_put(LED, 0);
        vTaskDelay(delay);
    }
}

// Static storage for the blink task
static StaticTask_t blink_tcb;
static StackType_t  blink_stack[256];

int main() {
    stdio_init_all();

    xTaskCreateStatic(
        blink_task,   // function
        "blink",      // name
        256,          // stack depth
        nullptr,      // params
        1,            // priority
        blink_stack,  // stack buffer
        &blink_tcb    // TCB buffer
    );

    vTaskStartScheduler();
}
