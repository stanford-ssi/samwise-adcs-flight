#include "apps/hitl_b/init.h"
#include "macros.h"

#include "hardware/timer.h"
#include "hardware/uart.h"

#include "drivers/software_uart/software_uart.h"

slate_t slate;

bool telem_timer_callback(struct repeating_timer *t)
{

    uint32_t chars = telemetry_read(&slate.telemetry, &slate.motor_uart);
    if (chars)
    {
        LOG_INFO("RX: Recieved chars: %d", chars);
    }

    return true;
}

int main()
{
    stdio_init_all();
    sleep_ms(1000);

    init(&slate);

    // Timer for telemetry sending
    add_repeating_timer_ms(255, telem_timer_callback, NULL, &slate.telem_timer);

    LOG_INFO("Initializing");
    sleep_ms(1000);

    while (1)
    {
    }
    ERROR("Yikes END OF CODE");
    return 0;
}
