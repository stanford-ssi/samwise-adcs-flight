#include "apps/motor_app/timer_util.h"

#include "hardware/timer.h"
#include "hardware/uart.h"
#include "apps/motor_app/motor_slate.h"

bool control_timer_callback(struct repeating_timer *t){ 
    float kp = 0.3f;

    for (int m = 1; m < 4; m++) {
        float diff = motor_slate.motor_state[1].rpm_ - motor_slate.motor_measured[1].rpm_;
        float du = kp*diff;

        motor_slate.motor_state[m].speed_ += (int)round(du);
        motor_slate.motor_state[m].speed_ &= 0b11111111111;
    }

    return true;

}

bool telem_timer_callback(struct repeating_timer *t) {
    printf("Sending telemetry\n");
    char* tx_bytes = (char*) &motor_slate.tx_package;
    for (int i = 0; i < sizeof(tx_package_t); i++){
        uart_putc(uart1, tx_bytes[i]);
    }

    return true;
}

