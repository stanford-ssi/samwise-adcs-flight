#include "hardware/timer.h"
#include "hardware/uart.h"

#include "apps/motor_app/motor_slate.h"
#include "apps/motor_app/timer_util.h"
#include "drivers/software_uart/software_uart.h"

bool control_timer_callback(struct repeating_timer *t)
{
    float kp = 0.3f;

    for (int m = 0; m < 4; m++)
    {
        float diff = motor_slate.motor_state[1].rpm_ -
                     motor_slate.motor_measured[1].rpm_;
        float du = kp * diff;

        motor_slate.motor_state[m].speed_ += (int)round(du);
        motor_slate.motor_state[m].speed_ &= 0b11111111111;
    }

    return true;
}

bool telem_timer_callback(struct repeating_timer *t)
{
    LOG_INFO("Sending telemetry package. Size: %d",
             sizeof(motor_to_adcs_package_t));

    uint8_t received = software_uart_rx_getbuf(&motor_slate.adcs_uart);
    // software_uart_putc(&motor_slate.adcs_uart, received);
    // uint8_t out[] = "hello world\r\n";
    // software_uart_putk(&motor_slate.adcs_uart, out);
    software_uart_tx_package<motor_to_adcs_package_t>(&motor_slate.adcs_uart,
                                                      &motor_slate.tx_package);

    return true;
}
