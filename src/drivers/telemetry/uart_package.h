#pragma once

#include "drivers/software_uart/software_uart.h"

#define START_FLAG 0xAA

struct base_telemetry_t {
    char flag = START_FLAG;
};

struct adcs_to_motor_package_t : base_telemetry_t {
    char flag = START_FLAG;
    bool motors_enabled[4];
    float target_rpm[4];
    uint32_t checksum;
};

struct motor_to_adcs_package_t: base_telemetry_t{
    char flag = START_FLAG;
    float battery_voltage;
    float battery_current;
    bool motors_alive[4];
    bool motors_data_valid[4];
    float measured_rpm[4];
    bool magnetometer_alive;
    bool magnetometer_data_valid;
    float b_body_raw[3];
    uint32_t checksum;
};

struct telemetry_handler_t {
    uint32_t frame_position;
    uint32_t frame_size;
    uint8_t* dest;
};

telemetry_handler_t telemetry_init(base_telemetry_t* dest);

/* Returns true if it read any uart bits, else returns false.
 */
bool telemetry_read(telemetry_handler_t* tel, software_uart_t* uart);

