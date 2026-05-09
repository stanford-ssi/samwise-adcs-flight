// #include "pico/printf.h"
#include "tusb.h"

#include "drivers/communications/protocol.h"
#include "drivers/communications/cobs.h"

void send_hitl(msg_t* msg, uint32_t len) {
    uint8_t msg_buf[len];
    protocol_message_encode(msg, msg_buf);
    uint8_t cobs_buf[len + 2];
    uint32_t end = cobs_encode(msg_buf, len, cobs_buf);
    cobs_buf[end] = 0;
    tud_cdc_write(cobs_buf, len + 2);
    tud_cdc_write("\0", 1);
    tud_cdc_write_flush();
}


