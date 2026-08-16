#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdio.h"

#include "pico/time.h"

#include "macros.h"

#include "apps/adcs_new/tasks/tasks.h"
#include "apps/adcs_new/params.h"
#include "apps/adcs_new/pins.h"
#include "apps/adcs_new/slate.h"

#include "drivers/picubed/picubed.h"
#include "drivers/communications/pio_uart_comms.h"

extern slate_t slate;

void vTaskTelemetry(void *) {
    int tx_count = 0;
    int rx_count = 0;
    static uint8_t rx_buf[256];
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_TELEMETRY))
        TASK_LOOP_MS(500)
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
                case MSG_COMMAND:
                    uint8_t command_byte = (uint32_t) received.payload;
                    LOG_INFO("[TELEMETRY] Command Received {%d}", command_byte);
                    StateMsg_t msg = MSG_COMMAND_RECEIVED;
                    xQueueSend(slate.state_machine.state_queue_handle,
                            &msg, 0);
                    break;
            } // end switch 
        } else {
        // SEND PING MESSAGE
            tx_count += 1;
            send_ping();
            adcs_packet_t adcs;
            adcs_packet_populate(&adcs,
                    slate.attitude_filter,
                    slate.gps_data,
                    slate.power_monitor,
                    slate.magnetometer_data,
                    slate.sun_sensor,
                    static_cast<uint8_t>(
                        slate.state_machine.current_state));
            send_adcs_packet(&adcs);
        }
    } // end for
}

void vTaskMotorRx(void *) {
    static uint8_t rx_buf[256];
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_MOTOR_BOARD))
        TASK_LOOP_MS(100)
        
        // TODO: this
        if (pio_uart_comms_packet_ready()) {
            LOG_INFO("[MOTOR] PACKET RECEIVED");
            msg_t received;
            // TODO: This
            // receive_msg(&received, rx_buf);
            switch (received.type) {
                case MSG_MOTOR_STATUS:
                    break;
                case MSG_PING:
                    //TODO: pong
                case MSG_PONG:
                    LOG_INFO("[MOTOR] Pong received");
                    break;
            } // end switch

        } // end if
    } // end for (;;)
}

void vTaskMotorTx(void *) {
    for (;;) {
        WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_MOTOR_BOARD))
        TASK_LOOP_MS(500)
        // TODO: This
        // motor_send_ctrl();
    }
}

