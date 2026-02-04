#include "macros.h"
#include "pico/time.h"

#include "apps/motor_app/irq.h"
#include "apps/motor_app/motor_slate.h"
#include "apps/motor_app/pins.h"
#include "drivers/software_uart/software_uart.h"

void gpio_irq(uint gpio, uint32_t events)
{
    int motor_id = 0;
    bool fgout = false;
    bool fault = false;

    if (gpio == MOTOR_0_FGOUT_PIN)
    {
        motor_id = 0;
        fgout = true;
    }
    if (gpio == MOTOR_1_FGOUT_PIN)
    {
        motor_id = 1;
        fgout = true;
    }
    if (gpio == MOTOR_2_FGOUT_PIN)
    {
        motor_id = 2;
        fgout = true;
    }
    if (gpio == MOTOR_3_FGOUT_PIN)
    {
        motor_id = 3;
        fgout = true;
    }

    if (gpio == MOTOR_0_FAULT_PIN)
    {
        motor_id = 0;
        fault = true;
    }
    if (gpio == MOTOR_1_FAULT_PIN)
    {
        motor_id = 1;
        fault = true;
    }
    if (gpio == MOTOR_2_FAULT_PIN)
    {
        motor_id = 2;
        fault = true;
    }
    if (gpio == MOTOR_3_FAULT_PIN)
    {
        motor_id = 3;
        fault = true;
    }

    if (gpio == ADCS_UART_COMM_RX)
    {
        software_uart_handle_rx_start(&motor_slate.adcs_uart);
        LOG_INFO("Receiving uart");
    }

    if (fgout)
    {
        absolute_time_t now = get_absolute_time();
        absolute_time_t prev =
            motor_slate.motor_measured[motor_id].last_pulse_time_;
        motor_slate.motor_measured[motor_id].last_pulse_time_ = now;

        // Compute instantaneous RPM
        if (!is_nil_time(prev))
        {
            int64_t dt_us = absolute_time_diff_us(prev, now); // microseconds
            float dt_s = dt_us / 1e6f;
            float rpm = 60.0f / (dt_s * PULSES_PER_REV);
            motor_slate.motor_measured[motor_id].rpm_ =
                (rpm > 6000) ? 6000 : rpm;
        }
    }

    if (fault)
    {
        ERROR("WOMP WOMP MOTOR FAULT\n");
    }
}
