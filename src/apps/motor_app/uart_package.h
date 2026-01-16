#pragma once

typedef struct {
    char rx_start_tag;
    bool motors_enabled[4];
    float target_rpm[4];
    uint32_t checksum;
} rx_package_t;

typedef struct {
    float battery_voltage;
    float battery_current;
    bool motors_alive[4];
    bool motors_data_valid[4];
    float measured_rpm[4];
    bool magnetometer_alive;
    bool magnetometer_data_valid;
    float b_body_raw[3];
    uint32_t checksum;
} tx_package_t;
