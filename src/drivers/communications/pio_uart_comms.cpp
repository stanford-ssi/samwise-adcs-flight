#include "pico/stdlib.h"
#include "pico/printf.h"
#include "hardware/pio.h"
#include "hardware/irq.h"

#include "macros.h"

#include "drivers/pio_uart/pio_uart.h"
#include "drivers/communications/uart_communications.h"

#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 256

#define DATA_BITS (8)
#define STOP_BITS (1)
#define PARITY    (UART_PARITY_NONE)

// Static PIO instances for RX and TX
static PIO pio_rx, pio_tx;
static uint8_t sm_rx, sm_tx;

// Ring buffers
static uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0, rx_tail = 0;

static uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint16_t tx_head = 0, tx_tail = 0;



// ── internal helpers ────────────────────────────────────────────────────────
static inline uint16_t rx_count() {
    return (rx_head - rx_tail + UART_RX_BUFFER_SIZE) % UART_RX_BUFFER_SIZE;
}

static inline uint16_t tx_count() {
    return (tx_head - tx_tail + UART_TX_BUFFER_SIZE) % UART_TX_BUFFER_SIZE;
}

static inline uint16_t tx_available() {
    return UART_TX_BUFFER_SIZE - 1 - tx_count();
}

// Peek at a byte at offset `offset` from rx_tail without consuming it
static inline uint8_t rx_peek(uint16_t offset) {
    return rx_buffer[(rx_tail + offset) % UART_RX_BUFFER_SIZE];
}

static void uart_rx_pio_isr() {
    while (!pio_sm_is_rx_fifo_empty(pio_rx, sm_rx)) {
        uint16_t next = (rx_head + 1) % UART_RX_BUFFER_SIZE;
        if (next == rx_tail) {
            (void)pio_sm_get(pio_rx, sm_rx); // drop byte
        } else {
            rx_buffer[rx_head] = (uint8_t)(pio_sm_get(pio_rx, sm_rx) >> 24);
            rx_head = next;
        }
    }
    pio_interrupt_clear(pio_rx, 0);
}

void pio_uart_comms_init(uint8_t tx, uint8_t rx, uint32_t baud) {
    rx_head = rx_tail = 0;
    tx_head = tx_tail = 0;

    pio_uart_rx_init(&pio_rx, &sm_rx, rx, baud);
    pio_uart_tx_init(&pio_tx, &sm_tx, tx, baud);

    // Set up RX interrupt
    uint irq_num;
    if      (pio_rx == pio0) irq_num = PIO0_IRQ_0;
    else if (pio_rx == pio1) irq_num = PIO1_IRQ_0;
    else                     irq_num = PIO2_IRQ_0;

    pio_set_irq0_source_enabled(pio_rx, pis_interrupt0, true);
    irq_set_priority(irq_num, 0x60);
    irq_set_exclusive_handler(irq_num, uart_rx_pio_isr);
    irq_set_enabled(irq_num, true);
}

uint16_t pio_uart_comms_tx(uint8_t *data, uint16_t length) {
    uint16_t avail = tx_available();
    if (length > avail) length = avail;

    for (uint16_t i = 0; i < length; i++) {
        tx_buffer[tx_head] = data[i];
        tx_head = (tx_head + 1) % UART_TX_BUFFER_SIZE;
    }

    // Drain TX buffer — PIO TX has no IRQ-driven drain like hardware UART,
    // so we push bytes directly. If blocking is a concern move this to a 
    // TX ISR or DMA instead.
    while (tx_tail != tx_head) {
        pio_uart_putc(tx_buffer[tx_tail]);
        tx_tail = (tx_tail + 1) % UART_TX_BUFFER_SIZE;
    }

    return length;
}

uint16_t pio_uart_comms_packet_ready() {
    uint16_t count = rx_count();
    for (uint16_t offset = 0; offset < count; offset++) {
        if (rx_peek(offset) == 0x00) {
            return offset + 1;
        }
    }
    return 0;
}

uint16_t pio_uart_comms_get_packet(uint8_t *buffer, uint16_t max_length) {
    uint16_t packet_length = pio_uart_comms_packet_ready();
    if (packet_length == 0) return 0;

    if (packet_length > max_length) {
        // Discard oversized packet
        for (uint16_t i = 0; i < packet_length; i++) {
            rx_tail = (rx_tail + 1) % UART_RX_BUFFER_SIZE;
        }
        return 0;
    }

    for (uint16_t i = 0; i < packet_length; i++) {
        buffer[i] = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % UART_RX_BUFFER_SIZE;
    }
    return packet_length;
}

