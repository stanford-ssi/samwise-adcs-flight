#pragma once
/*
 * Author:      @Carson Lauer
 * Date:        April 26, 2026
 * Description: Communication protocol header.
 * Includes a protocol to communicate
 */

#include <stdint.h>

#include "adcs_packet.h"

enum {
    MSG_PING,           // No-op, but returns a pong
    MSG_PONG,           
    MSG_STRING,
    MSG_FLOAT3,
    MSG_COMMAND,        // Execute command
    MSG_ADCS_PACKET,     // ADCS 
    MSG_MOTOR_CTRL,
    MSG_MOTOR_STATUS
};

/*
 * One-byte payload values for MSG_COMMAND.
 *
 * Keep these values in sync with the PiCubed command encoder. Commands select
 * an explicit mode; they are not toggles, so retries are idempotent.
 */
typedef enum : uint8_t {
    ADCS_COMMAND_DISABLE = 0,
    ADCS_COMMAND_ENABLE_NORMAL = 1,
    ADCS_COMMAND_ENTER_DETUMBLE = 2,
} AdcsCommand_t;

struct __attribute((packed)) msg_t {
    uint8_t src;
    uint8_t dst;
    uint8_t seq;
    uint8_t flags;
    uint8_t type;
    uint8_t len;
    uint8_t *payload;
    uint8_t crc8;
};

struct __attribute((packed)) motor_ctrl_t {
    uint32_t state_override;
    bool motor_enabled[4];
    float target_rpm[4];
};

struct __attribute((packed)) motor_status_t {
    float battery_voltage;
    float battery_current;
    bool motors_alive[4];
    bool motors_data_valid[4];
    float measured_rpm[4];
    bool magnetometer_alive;
    bool magnetometer_data_valid;
    float b_body_raw[3];
} ;

/*
 * Creates a protocol ping message
 */
void protocol_message_ping(msg_t *msg);

void protocol_message_pong(msg_t *msg);

void protocol_message_float3(msg_t *msg, float *data);

void protocol_message_string(msg_t *msg, uint8_t* s);

uint32_t protocol_message_adcs(msg_t *msg, adcs_packet_t* adcs);

 /*
 * Takes a message and formats it into a buffer
 */
void protocol_message_encode(msg_t *msg, uint8_t *buf);

bool protocol_message_decode(msg_t *msg, uint32_t len, uint8_t *buf);
