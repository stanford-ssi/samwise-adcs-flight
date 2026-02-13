#include "drivers/telemetry/uart_package.h"

#include "macros.h"

telemetry_handler_t telemetry_init(base_telemetry_t *dest)
{
    telemetry_handler_t out;
    out.frame_position = 0;
    out.frame_size = sizeof(*dest);
    out.dest = (uint8_t *)dest;
    return out;
}

uint32_t telemetry_read(telemetry_handler_t *tel, software_uart_t *uart)
{
    if (!software_uart_is_readable(uart))
    {
        return false;
    }
    uint32_t chars_received = 0;
    while (software_uart_is_readable(uart))
    {
        uint8_t c = software_uart_rx_getbuf(uart);
        chars_received++;
        // LOG_INFO("Character received: %x", c);
        //  detect start flag
        if (c == START_FLAG)
        {
            // Set frame position to 1 (first bit after flag)
            tel->frame_position = 1;
            continue;
        }
        // if not start flag just set to the position in struct
        tel->dest[tel->frame_position] = c;
        tel->frame_position++;
        tel->frame_position %= tel->frame_size;
    }
    return chars_received;
}
