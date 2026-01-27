#include "apps/hitl_b/init.h"
#include "apps/hitl_b/slate.h"
#include "apps/hitl_b/pins.h"

#include "macros.h"

#include "drivers/telemetry/uart_package.h"
#include "drivers/software_uart/software_uart.h"

void gpio_irq(uint gpio, uint32_t events) {

    if (gpio == HITL_RX) {
        software_uart_handle_rx_start(&slate.motor_uart);
        LOG_INFO("Receiving uart");
    }

}

int init(slate_t* slate) {
    slate->telemetry = telemetry_init(&slate->rx_package);

    gpio_set_irq_enabled_with_callback(HITL_RX, 
            GPIO_IRQ_EDGE_FALL, 
            true, 
            &gpio_irq);

    slate->motor_uart = software_uart_init(HITL_RX, HITL_TX);

    return 0;
}
