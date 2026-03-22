#include "apps/hitl_a/init.h"
#include "apps/hitl_a/pins.h"
#include "apps/hitl_a/slate.h"

#include "drivers/software_uart/software_uart.h"
#include "drivers/telemetry/uart_package.h"

int init(slate_t *slate)
{

    slate->telemetry = telemetry_init(&slate->rx_package);

    slate->adcs_uart = software_uart_init(HITL_RX, HITL_TX);

    return 0;
}
