/* Author: @Carson Lauer
 * Last Updated: January 26, 2026
 * This board acts as an adcs board mimic.
 * It has a control loop that sends out data
 * packets every second.
 */

#include "hardware/timer.h"
#include "hardware/uart.h"

#include "drivers/software_uart/software_uart.h"

#include "apps/hitl_a/init.h"
#include "macros.h"

slate_t slate;

bool telem_timer_callback(struct repeating_timer *t) {
    LOG_INFO("Sending telemetry package. Size: %d", sizeof(slate.tx_package));

    software_uart_tx_package<adcs_to_motor_package_t>(
            &slate.adcs_uart, 
            &slate.tx_package);
    
    return true;
}


int main(){
    stdio_init_all();
    sleep_ms(1000);

    init(&slate);
    sleep_ms(1000);

    // Timer for telemetry sending
    add_repeating_timer_ms(1000, 
            telem_timer_callback, 
            NULL, 
            &slate.telem_timer);

    LOG_INFO("Initializing");
    sleep_ms(1000);

    while(1){}
    ERROR("Yikes END OF CODE");

    return 0;
}
