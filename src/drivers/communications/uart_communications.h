#pragma once
/*
 * Author:      @Carson Lauer
 * Date:        April 26, 2026
 * Description: Uart communication library.
 * This library includes a irq_callback to tx uart messages from a buffer
 * and to receive uart messages into a buffer. It assumes either COBS
 * encoded packets or null-terminated strings (both end in an 0x00 char).
 */

#include <stdint.h>

void uart_comms_init(uint32_t uart_instance, 
        uint8_t tx, 
        uint8_t rx, 
        uint32_t baud);
