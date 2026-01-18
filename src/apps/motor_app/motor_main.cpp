/**
**
 * @author Carson Lauer
 *
 * This is the main motor file
 */

#include "pico/float.h"
#include "pico/stdlib.h"

#include "hardware/gpio.h"

#include "macros.h"

#include "drivers/motor/motor.h"

#include "apps/motor_app/pins.h"
#include "apps/motor_app/init.h"
#include "apps/motor_app/motor_slate.h"
#include "apps/motor_app/timer_util.h"

// Make sure top gpio bank enabled
static_assert(PICO_RP2350A == 0,
              "PICO_RP2350A must be defined to 0 for PICUBED builds.");

motor_slate_t motor_slate;

int main()
{
    stdio_init_all();
    sleep_ms(1000);

    // Initialize
    init(&motor_slate);

    // Initialize motors
    for (int m = 0; m < 4; m++){
        motor_enable(&motor_slate.motors[m]);

        motor_slate.rx_package.target_rpm[m] = 2000.f;
        motor_slate.motor_state[m].rpm_ = 2000.f;

        motor_slate.motor_state[m].enabled_ = 1;
    }

    int period_ms = 100;

    // Timer for motor control loop
    add_repeating_timer_ms(period_ms,
                        control_timer_callback, 
                        NULL, 
                        &motor_slate.control_timer);

    // Timer for telemetry sending
    add_repeating_timer_ms(1000, 
            telem_timer_callback, 
            NULL, 
            &motor_slate.telem_timer);

    sleep_ms(1000);

    //char* rx_bytes = (char*) &motor_slate.rx_package;
    //int rx_count = 0;

    while (1)
    {
        // Feed watchdog
        watchdog_feed(&motor_slate.watchdog);

        // Read voltage and current
        float v = adm1176_get_voltage(&motor_slate.power_monitor);
        float c = adm1176_get_current(&motor_slate.power_monitor);

        motor_slate.tx_package.battery_voltage = v;
        motor_slate.tx_package.battery_current = c;

        motor_slate.tx_package.checksum = 0x11223344;

        // Output voltage
        LOG_INFO("Voltage: %f", v);
        LOG_INFO("Current: %f", c);

        /*
        if (uart_is_readable(uart1)) {
            char read = uart_getc(uart1);
           rx_bytes[rx_count] = read; 
           rx_count += 1;
           rx_count %= sizeof(rx_package_t);
        }
        */

        for (int m = 0; m < 4; m++) {
            motor_slate.motor_state[m].rpm_ = motor_slate.rx_package.target_rpm[m];
            if (motor_slate.motor_state[m].enabled_) {
                // printf("Setting motor %d\n", m);
                int motor_speed = motor_slate.motor_state[m].speed_;
                motor_set_speed(&motor_slate.motors[m], motor_speed);
            }
        }
        sleep_ms(10);
    }

	ERROR("END OF PROGRAM REACHED (BAD)");

    while (1)
        ;
}
