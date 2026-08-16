/**
 * @author Niklas Vainio
 * @date 2025-05-24
 *
 * This file contains functions for interacting with the PiCubed over UART
 */
#pragma once

#include "adcs_packet.h"

#include "gnc/mekf/filter.h"
#include "drivers/gps/gps.h"
#include "drivers/power_monitor/power_monitor.h"
#include "drivers/magnetometer/magnetometer.h"
#include "drivers/sun_sensors/sun_sensors.h"

#include "drivers/communications/cobs.h"
#include "drivers/communications/protocol.h"
#include "drivers/communications/uart_communications.h"

void picubed_uart_init();

bool picubed_uart_handle_commands();

void receive_msg(msg_t *msg, uint8_t *rx_buf);

void send_msg(msg_t *msg, uint32_t len); 

void send_ping();

void send_pong();

void adcs_packet_populate(adcs_packet_t* adcs,
    AttitudeFilter &attitude,
    gps_data_processed_t &gps,
    power_monitor_t &power,
    MagnetometerData &mag,
    sun_sensor_data_t &sun,
    uint8_t state);

void send_adcs_packet(adcs_packet_t* adcs);
