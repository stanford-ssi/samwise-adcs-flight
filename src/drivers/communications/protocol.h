#pragma once
/*
 * Author:      @Carson Lauer
 * Date:        April 26, 2026
 * Description: Communication protocol header.
 * Includes a protocol to communicate
 */

#include <stdint.h>

#include "drivers/telemetry/uart_package.h"

enum {
    MSG_PING,           // No-op, but returns a pong
    MSG_PONG,           
    MSG_COMMAND,        // Execute command
    MSG_ADCS_PACKET     // ADCS 
};

typedef struct protocol_msg {
    uint8_t src;
    uint8_t dst;
    uint8_t seq;
    uint8_t flags;
    uint8_t message_type;
    uint8_t payload_len;
    uint8_t *payload;
    uint8_t crc8;
} msg_t;

/*
 * Creates a protocol ping message
 */
uint32_t protocol_message_ping(msg_t *msg);

uint32_t protocol_message_pong(msg_t *msg);

uint32_t protocol_message_string(msg_t *msg, uint8_t* s);

uint32_t protocol_message_adcs(msg_t *msg, adcs_message_t* adcs);

 /*
 * Takes a message and formats it into a buffer
 */
uint32_t protocol_message_buf(msg_t *msg, void *buf);


