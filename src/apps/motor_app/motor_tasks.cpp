#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "drivers/motor/motor.h"
#include "apps/motor_app/motor_slate.h"
#include "apps/motor_app/motor_tasks.h"
#include "apps/motor_app/state_machine.h"

extern motor_slate_t motor_slate;

void telemetry_task(void *) {
    constexpr TickType_t delay = pdMS_TO_TICKS(3000);

    uint32_t count = 0;

    while (true) {
        xEventGroupWaitBits(motor_slate.events, 
            TASK_BIT(TASK_TELEMETRY), pdFALSE, pdTRUE, portMAX_DELAY);
 
        StateMsg_t msg = MSG_IDLE;
        switch (motor_slate.current_state) {
            case STATE_ENABLED:
                LOG_INFO("In enabled state");
                if (count % 5 == 0)
                    xQueueSend(motor_slate.state_queue_handle,
                            &msg, 0);
                break;
            case STATE_SAFE:
                LOG_INFO("In safe state");
                if (count % 5 == 0)
                    xQueueSend(motor_slate.state_queue_handle,
                            &msg, 0);
                break;
        };
        LOG_INFO("Sending telemetry package. Size: %d",
                 sizeof(motor_to_adcs_package_t));

        uint8_t received = software_uart_rx_getbuf(&motor_slate.adcs_uart);

        software_uart_tx_package<motor_to_adcs_package_t>(&motor_slate.adcs_uart,
                                                          &motor_slate.tx_package);

        // for (int m = 0; m < 4; m++) {
        //     motor_slate.rx_package.target_rpm[m] = 3000.f;
        // }

        // Update target rpm based on controls set by telemetry
        for (int m = 0; m < 4; m++) {
            motor_slate.motor_state[m].rpm_ =
                motor_slate.rx_package.target_rpm[m];
        }

        count++;
        vTaskDelay(delay);
    }
}

void power_monitor_task(void *) {


    while (true) {
        xEventGroupWaitBits(motor_slate.events, 
                TASK_BIT(TASK_POWER), pdFALSE, pdTRUE, portMAX_DELAY);

        TickType_t xLastWakeTime = xTaskGetTickCount();
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000)); 

        // Read voltage and current
        float v = adm1176_get_voltage(&motor_slate.power_monitor);
        float c = adm1176_get_current(&motor_slate.power_monitor);

        motor_slate.tx_package.battery_voltage = v;
        motor_slate.tx_package.battery_current = c;

        motor_slate.tx_package.checksum = 0x11223344;

        // Output voltage
        LOG_INFO("Voltage: %f", v);
        LOG_INFO("Current: %f", c);

    }
}

void watchdog_feed_task(void *) {
    while(1) {
        xEventGroupWaitBits(motor_slate.events, 
            TASK_BIT(TASK_WATCHDOG), pdFALSE, pdTRUE, portMAX_DELAY);
        // LOG_INFO("Watchdog fed");
        watchdog_feed(&motor_slate.watchdog);
    }
}

void control_task(void *) {
    float kp = 0.3f;


    while(1) {
        xEventGroupWaitBits(motor_slate.events, 
            TASK_BIT(TASK_CONTROL), pdFALSE, pdTRUE, portMAX_DELAY);
        TickType_t xLastWakeTime = xTaskGetTickCount();
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(300));

        LOG_INFO("Controlling!");
        for (int m = 0; m < 4; m++) {
            float diff = motor_slate.motor_state[1].rpm_ -
                         motor_slate.motor_measured[1].rpm_;
            float du = kp * diff;

            // Set target rpm
            int int_diff = (int)round(du) & 0x1FFFFFF;
            motor_slate.motor_state[m].speed_ += int_diff;
            int motor_speed = motor_slate.motor_state[m].speed_; 
            // If motors on, apply speed
            if (motor_slate.motor_state[m].enabled_) {
                motor_set_speed(&motor_slate.motors[m], motor_speed);
            }
        }
    } 
}
