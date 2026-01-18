#include "pico/printf.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"

#include "macros.h"
#include "pins.h"
#include "motor_slate.h"

#include "apps/motor_app/init.h"
#include "apps/motor_app/irq.h"
#include "drivers/software_uart/software_uart.h"
#include "drivers/adm1176/adm1176.h"
#include "drivers/watchdog_motor/watchdog.h"
#include "drivers/motor/motor.h"

static bool init_gpio_pins(motor_slate_t *motor_slate){
    i2c_init(SAMWISE_POWER_MONITOR_I2C, 100 * 1000);
    gpio_set_function(SAMWISE_POWER_MONITOR_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SAMWISE_POWER_MONITOR_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SAMWISE_POWER_MONITOR_SDA_PIN);
    gpio_pull_up(SAMWISE_POWER_MONITOR_SCL_PIN);
	return true;
}

static bool init_drivers(motor_slate_t *motor_slate){
	adm1176_t power_monitor = adm1176_mk(SAMWISE_POWER_MONITOR_I2C, 
					ADM1176_I2C_ADDR,
					ADM1176_DEFAULT_SENSE_RESISTOR);

    adm1176_on(&power_monitor);

   	motor_slate->power_monitor = power_monitor;

	return true;
}

static bool init_motors(motor_slate_t *motor_slate) {
	motor_slate->motors[0] = motor_init(MOTOR_0_FAULT_PIN,
			MOTOR_0_SLEEP_PIN,
			MOTOR_0_DRVOFF_PIN,
			MOTOR_0_SCS_PIN,
			MOTOR_0_BRAKE_PIN,
			MOTOR_0_PWM_PIN,
			MOTOR_0_FGOUT_PIN);

	motor_slate->motors[1] = motor_init(MOTOR_1_FAULT_PIN,
			MOTOR_1_SLEEP_PIN,
			MOTOR_1_DRVOFF_PIN,
			MOTOR_1_SCS_PIN,
			MOTOR_1_BRAKE_PIN,
			MOTOR_1_PWM_PIN,
			MOTOR_1_FGOUT_PIN);

	motor_slate->motors[2] = motor_init(MOTOR_2_FAULT_PIN,
			MOTOR_2_SLEEP_PIN,
			MOTOR_2_DRVOFF_PIN,
			MOTOR_2_SCS_PIN,
			MOTOR_2_BRAKE_PIN,
			MOTOR_2_PWM_PIN,
			MOTOR_2_FGOUT_PIN);

	motor_slate->motors[3] = motor_init(MOTOR_3_FAULT_PIN,
			MOTOR_3_SLEEP_PIN,
			MOTOR_3_DRVOFF_PIN,
			MOTOR_3_SCS_PIN,
			MOTOR_3_BRAKE_PIN,
			MOTOR_3_PWM_PIN,
			MOTOR_3_FGOUT_PIN);

    for (int i = 0; i < 4; i++) 
    {
        motor_t* motor = &motor_slate->motors[i];
        gpio_set_irq_enabled_with_callback(motor->FGOUT_pin_, GPIO_IRQ_EDGE_FALL, true, &gpio_irq);
        gpio_set_irq_enabled_with_callback(motor->FAULT_pin_, GPIO_IRQ_EDGE_FALL, true, &gpio_irq);
    }


	return true;
}

static bool init_spi(motor_slate_t* motor_slate) {
	// SPI
	spi_init(spi0, 1000 * 1000);
	spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(ADCS_MOTOR_SDO, GPIO_FUNC_SPI);
    gpio_set_function(ADCS_MOTOR_SDI, GPIO_FUNC_SPI);
    gpio_set_function(ADCS_MOTOR_SCLK, GPIO_FUNC_SPI);

	return true;
}
/*
void uart_rx_read(){
    char* rx_bytes = (char*) &motor_slate.rx_package;
    printf("uart interrupt\n");
    while (uart_is_readable(uart1)) {
        char read = uart_getc(uart1);
        uart_putc(uart1, read);
        rx_bytes[motor_slate.rx_count] = read; 
        motor_slate.rx_count += 1;
        motor_slate.rx_count %= sizeof(rx_package_t);
    }
}
*/

static bool init_uart(motor_slate_t* motor_slate) {
    /*
    uart_init(uart1, 115200);

    gpio_set_function(ADCS_UART_COMM_TX, UART_FUNCSEL_NUM(uart1, 0)); // TX
    gpio_set_function(ADCS_UART_COMM_RX, UART_FUNCSEL_NUM(uart1, 1)); // RX
                                                                      //
    // Disable FIFO to get interrupt per character 
    uart_set_fifo_enabled(uart1, false);

    irq_set_exclusive_handler(UART1_IRQ, uart_rx_read);
    irq_set_enabled(UART1_IRQ, true);

    uart_set_irqs_enabled(uart1, true, false);
    */
    
    // Initialise UART 1
    gpio_set_irq_enabled_with_callback(ADCS_UART_COMM_RX, GPIO_IRQ_EDGE_FALL, true, &gpio_irq);
    motor_slate->adcs_uart = software_uart_init(ADCS_UART_COMM_RX, ADCS_UART_COMM_TX);

    return true;
}

static bool init_watchdog() {
    motor_slate.watchdog = watchdog_mk(SAMWISE_WATCHDOG_FEED_PIN);
    watchdog_init(&motor_slate.watchdog);
    return true;
}

bool init(motor_slate_t *motor_slate) {
	printf("Initializing...\n");
    gpio_init(SAMWISE_WATCHDOG_FEED_PIN);

    ASSERT(init_watchdog());

	ASSERT(init_gpio_pins(motor_slate));
    gpio_set_dir(SAMWISE_WATCHDOG_FEED_PIN, GPIO_OUT);

	ASSERT(init_spi(motor_slate));

    ASSERT(init_uart(motor_slate));

	ASSERT(init_motors(motor_slate));

	ASSERT(init_drivers(motor_slate));

	return true;
}
