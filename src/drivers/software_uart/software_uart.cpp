#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/platform.h"

#include "macros.h"

#include "hardware/timer.h"
#include "hardware/gpio.h"

#include "drivers/software_uart/software_uart.h"

#define BIT_TIME_US (1000000 / 9600)


// Initialize with more detailed settings
software_uart_t software_uart_init(uint32_t rx, uint32_t tx, uint32_t baud, uint32_t parity) {
    software_uart_t out;
    out.rx = rx;
    out.tx = tx;
    out.baud = baud;
    out.parity = parity;
    out.tx_bit = 0;
    out.rx_bit = 0;

    out.tx_buffer_start = 0;
    out.tx_buffer_end = 0;

    out.rx_buffer_start = 0;
    out.rx_buffer_end = 0;

    gpio_init(tx);
    gpio_set_function(tx, GPIO_FUNC_SIO);
	gpio_disable_pulls(tx);
	gpio_set_dir(tx, GPIO_OUT);
	gpio_put(tx, 1);

    gpio_init(rx);
    gpio_set_dir(rx, GPIO_IN);

    return out;
}


software_uart_t software_uart_init(uint32_t rx, uint32_t tx) {
    software_uart_t out = software_uart_init(rx, tx, 9600, 0);
    return out;
}

// Set the baud rate on a software uart handler
void software_uart_set_baud(software_uart_t* target, uint32_t baud) {
    target->baud = baud;
}

// Set the parity on a software uart handler
void software_uart_set_parity(software_uart_t* target, uint32_t parity) {
    if((parity & 0b11) != parity) // Parity must be either 0, 1, or 2
        return;
    target->parity = parity;

}

// Handle uart rx
int64_t software_uart_handle_rx(alarm_id_t id, void *user_data) {
    software_uart_t *uart = (software_uart_t *)user_data;

    if (uart->rx_bit < 8) {
        int bit = gpio_get(uart->rx);
        uart->rx_byte |= (bit << uart->rx_bit);
        uart->rx_bit++;
        return BIT_TIME_US;
    }

    // Stop bit (ignored, but could check HIGH)
    uart->receiving = false;
    uart->rx_buffer[uart->rx_buffer_end] = uart->rx_byte;
    uart->rx_buffer_end++;
    uart->rx_buffer_end %= BUFFER_SIZE;

    return 0; // Stop alarm
};

// Handle uart rx
void software_uart_handle_rx_start(software_uart_t* uart) {
    if (uart->receiving) return;

    uart->receiving = true;
    uart->rx_bit = 0;
    uart->rx_byte = 0;

    // Sample in the *middle* of the first data bit
    uart->rx_alarm_id = add_alarm_in_us(
            BIT_TIME_US + BIT_TIME_US / 2,
            software_uart_handle_rx,
            uart,
            false
            );
}

int64_t software_uart_handle_tx(alarm_id_t id, void *user_data) {
    software_uart_t *uart = (software_uart_t*) user_data;

    if (uart->tx_bit == 0) {
        gpio_put(uart->tx, 1); // Idle high
        uart->transmitting = false;
        if(uart->tx_buffer_start != uart->tx_buffer_end){
            software_uart_handle_tx_start(uart);
        }
        return 0;
    }

    gpio_put(uart->tx, uart->tx_byte & 1);
    uart->tx_byte >>= 1;
    uart->tx_bit--;
    
    return BIT_TIME_US;
}

void software_uart_handle_tx_start(software_uart_t *uart){
    while (uart->transmitting) return;

    uint8_t byte = uart->tx_buffer[uart->tx_buffer_start];
    uart->tx_buffer_start++;
    uart->tx_buffer_start %= BUFFER_SIZE;

    uart->tx_byte = (1 << 8) | byte;
    uart->tx_bit = 9;
    uart->transmitting = true;

    gpio_put(uart->tx, 0);

    uart->tx_alarm_id = add_alarm_in_us(
            BIT_TIME_US,
            software_uart_handle_tx,
            uart,
            false
            );

    return;
}

void software_uart_putc(software_uart_t* uart, uint8_t c) {
    software_uart_tx_putbuf(uart, c);
    software_uart_handle_tx_start(uart);
}

// Print null terminated string
void software_uart_putk(software_uart_t* uart, uint8_t* s){
    while (*s != 0){
        software_uart_tx_putbuf(uart, *s);
        s++;
    }
    software_uart_handle_tx_start(uart);
}

// Put char to end of tx buffer
void software_uart_tx_putbuf(software_uart_t* uart, uint8_t byte) {
    uart->tx_buffer[uart->tx_buffer_end] = byte;
    uart->tx_buffer_end++;
    uart->tx_buffer_end %= BUFFER_SIZE;
}

// Get top of rx buffer
uint8_t software_uart_rx_getbuf(software_uart_t* uart) {
    if (uart->rx_buffer_start == uart->rx_buffer_end) return 0;
    uint8_t out = uart->rx_buffer[uart->rx_buffer_start];
    uart->rx_buffer_start++;
    uart->rx_buffer_start %= BUFFER_SIZE;
    return out; 
}

// check if uart i- readable
bool software_uart_is_readable(software_uart_t* uart) {
    bool readable = (uart->rx_buffer_start != uart->rx_buffer_end);
    return readable;
}


