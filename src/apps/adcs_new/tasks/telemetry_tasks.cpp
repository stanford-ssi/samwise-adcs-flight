#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdio.h"

#include "pico/time.h"

#include "macros.h"

#include "apps/adcs_new/tasks/tasks.h"
#include "apps/adcs_new/params.h"
#include "apps/adcs_new/pins.h"
#include "apps/adcs_new/slate.h"

#include "drivers/communications/cobs.h"
#include "drivers/communications/protocol.h"
#include "drivers/communications/uart_communications.h"

extern slate_t slate;

static void receive_msg(msg_t *msg, uint8_t *rx_buf) {
    static uint8_t raw_buf[256];
    uint16_t num_bytes = uart_comms_get_packet(SAMWISE_ADCS_PICUBED_UART,
            raw_buf, 256);
    cobs_decode(raw_buf, num_bytes, rx_buf);
    protocol_message_decode(msg, num_bytes + 1, rx_buf);
}

static void send_msg(msg_t *msg, uint32_t len) {
    uint8_t msg_buf[len];
    protocol_message_encode(msg, msg_buf);
    uint8_t cobs_buf[len + 2];
    uint32_t end = cobs_encode(msg_buf, len, cobs_buf);
    cobs_buf[end] = 0;
    uart_comms_tx(SAMWISE_ADCS_PICUBED_UART, cobs_buf, end + 1);
}

static void send_ping(){
    LOG_INFO("[TELEMETRY] Sending Ping");
    msg_t ping;
    protocol_message_ping(&ping);
    send_msg(&ping, 8);
}

static void send_pong(){
    LOG_INFO("[TELEMETRY] Sending Pong");
    msg_t pong;
    protocol_message_pong(&pong);
    send_msg(&pong, 8);
}



void vTaskTelemetry(void *) {
    int tx_count = 0;
    int rx_count = 0;
    static uint8_t rx_buf[256];
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_TELEMETRY));
        TASK_LOOP_MS(500);
        LOG_INFO("TX_COUNT {%d}", tx_count);

        if(uart_comms_packet_ready(SAMWISE_ADCS_PICUBED_UART)) {
            LOG_INFO("[TELEMETRY] PACKET RECEIVED {%d}", rx_count);
            rx_count += 1;
            msg_t received;
            receive_msg(&received, rx_buf);
            switch (received.type) {
                case MSG_PING:
                    LOG_INFO("[TELEMETRY] Ping received");
                    send_pong();
                    break;
                case MSG_PONG:
                    LOG_INFO("[TELEMETRY] Pong received");
                    // don't send ping else we get infinite loop
                    break;
            } // end switch 
        } else {
        // SEND PING MESSAGE
            tx_count += 1;
            send_ping();
        }
    } // end for
}

