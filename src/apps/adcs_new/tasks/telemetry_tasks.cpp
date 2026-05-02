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

void vTaskTelemetry(void *) {
    int tx_count = 0;
    int rx_count = 0;
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_TELEMETRY));
        TASK_LOOP_MS(200);
        // SEND PING MESSAGE
        LOG_INFO("[TELEMETRY] Task running");
        msg_t ping;
        protocol_message_ping(&ping);
        uint8_t buf[8];
        protocol_message_buf(&ping, buf);
        uint8_t cobs_buf[16];
        uint32_t end = cobs_encode(buf, 8, cobs_buf);
        cobs_buf[end] = 0;
        // LOG_INFO("[COBS]:");
        // for (int i = 0; i < 10; i++) {
        //     printf("[%d] ", cobs_buf[i]);
        // }
        // tx_count += 1;
        // printf("\n);
        tx_count += 1;
        LOG_INFO("TX_COUNT {%d}", tx_count);

        // In your task loop, before packet_ready check:
        LOG_INFO("[TELEMETRY] RX bytes in buffer: %d", 
            uart_comms_rx_count(SAMWISE_ADCS_PICUBED_UART));

        uart_putc(uart1, 0xAA);

        uart_comms_tx(SAMWISE_ADCS_PICUBED_UART,
                cobs_buf,
                end + 1);

        if(uart_comms_packet_ready(SAMWISE_ADCS_PICUBED_UART)) {
            rx_count += 1;
            LOG_INFO("[TELEMETRY] PACKET RECEIVED {%d}", rx_count);
        }
    }
}

