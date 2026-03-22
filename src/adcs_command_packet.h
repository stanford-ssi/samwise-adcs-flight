#pragma once

#include <stdint.h>

// ======================= ADCS Command Packet Structure (KEEP UP TO DATE WITH PICUBED) =========================

#define MAX_DATA_BYTES (250)

typedef enum{ //Add commands here.
    SEND_TELEM = 'T',
    HEALTH_CHECK = '?'
} adcs_tx_command;

typedef struct __attribute__((packed))
{
    adcs_tx_command command; 
    uint8_t packet_length;

    uint8_t packet_data[MAX_DATA_BYTES]; // in the respective functions, queue the data!
    //MAX_DATA_BYTES will have to be the field with the maximum required bytes.
} adcs_command_packet;



// ==============================================================================================================