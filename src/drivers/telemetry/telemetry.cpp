#include "drivers/telemetry/uart_package.h"

telemetry_handler_t telemetry_init(base_telemetry_t* dest) {
    telemetry_handler_t out;
    out.frame_position = 0;
    out.frame_size = sizeof(*dest);
    out.dest = (uint8_t*) dest;
    return out;
}

bool telemetry_read(telemetry_handler_t* tel, software_uart_t* uart){
    if(!software_uart_is_readable(uart)){
        return false;
    }
    while(software_uart_is_readable(uart)){
        uint8_t c = software_uart_rx_getbuf(uart);
        // detect start flag
        if (c == START_FLAG ) {
            tel->frame_position = 0;
            tel->frame_position++;
            continue;
        }
        // if not start flag just set to the position in struct
        tel->dest[tel->frame_position] = c;
        tel->frame_position++;
        tel->frame_position %= tel->frame_size;
    }
    return true;
}

