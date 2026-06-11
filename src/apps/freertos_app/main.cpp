#include <cstdio>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "rtos_setup.h"

#include "tusb.h"

// Static storage for the blink task
static StaticTask_t blink_tcb;
static StackType_t  blink_stack[256];

// logging tasks
static StaticTask_t log1_tcb;
static StackType_t log1_stack[256];

static StaticTask_t log2_tcb;
static StackType_t log2_stack[256];

static StaticSemaphore_t usb_semaphore_buf;
static SemaphoreHandle_t usb_mutex;

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

void vLog1Task(void *) 
{
    uint8_t buf[13] = "0123456789\n\r";
    for (;;) {
        if (xSemaphoreTake(usb_mutex, portMAX_DELAY) == pdTRUE ) 
        {
            if (tud_cdc_write_available() >= 12) {
                tud_cdc_write(buf, 12);
                tud_cdc_write_flush();
            }
            xSemaphoreGive(usb_mutex);
        } // end mutex
        vTaskDelay(pdMS_TO_TICKS(1));
    } // end for(;;)
}


void vLog2Task(void *) 
{
    uint8_t buf[9] = "abcdef\n\r";
    for (;;) {
        if (xSemaphoreTake(usb_mutex, portMAX_DELAY) == pdTRUE ) 
        {
            if (tud_cdc_write_available() >= 8) {
                tud_cdc_write(buf, 8);
                tud_cdc_write_flush();
            }
 
            tud_cdc_write_flush();
            xSemaphoreGive(usb_mutex);
        } // end mutex
        vTaskDelay(pdMS_TO_TICKS(1));
    } // end for(;;)
}

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

    xTaskCreateStatic(
        vLog1Task,   // function
        "log 1",      // name
        256,          // stack depth
        nullptr,      // params
        1,            // priority
        log1_stack,  // stack buffer
        &log1_tcb    // TCB buffer
    );

    xTaskCreateStatic(
        vLog2Task,   // function
        "log 2",      // name
        256,          // stack depth
        nullptr,      // params
        1,            // priority
        log2_stack,  // stack buffer
        &log2_tcb    // TCB buffer
    );

    usb_mutex = xSemaphoreCreateMutexStatic( &usb_semaphore_buf);

    vTaskStartScheduler();
}
