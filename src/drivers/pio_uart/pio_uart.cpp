#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "uart_tx.pio.h"
#include "uart_rx.pio.h"

static uint offset_rx;

static uint offset_tx;

// -- RX --
bool pio_uart_rx_init(PIO *pio_rx, uint8_t *sm_rx, uint8_t pin, uint32_t baud)
{
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&uart_rx_program, pio_rx, sm_rx, &offset_rx, pin, 1, true);
    if (success) {
        uart_rx_program_init(pio_rx, sm_rx, offset_rx, pin, baud);
    }
    return success;
}

uint8_t pio_uart_getc() 
{
    return uart_rx_program_getc(pio_rx, sm_rx);;
}

// -- TX --
bool pio_uart_tx_init(PIO *pio_rx, uint8_t *sm_tx, uint8_t pin_tx, uint32_t baud)
{
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&uart_tx_program, pio_tx, sm_tx, &offset_tx, pin_tx, 1, true);
    if (success) {
        uart_tx_program_init(pio_tx, sm_tx, offset_tx, pin_tx, baud);
    }
    return success;
}

void pio_uart_putc(PIO pio_rx, uint8_t sm_tx, char c) 
{
    uart_tx_program_putc(pio_tx, sm_tx, c);
}
void pio_uart_puts(const char* s) 
{
    while (*s) {
        uart_tx_program_putc(pio_tx, sm_tx, *s++);
    }
}
