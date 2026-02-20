/**
 * @author Niklas Vainio
 * @date 2025-05-24
 *
 * This file contains functions for interacting with the PiCubed over UART
 */

#include "hardware/flash.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#include "macros.h"
#include "picubed.h"
#include "pins.h"
#include "adcs_command_packet.h"
#include "pico/util/queue.h"
#include <stdint.h>
#include <cstring>


// Uart parameters
#define PICUBED_UART_BAUD (115200)
#define PICUBED_UART_DATA_BITS (8)
#define PICUBED_UART_STOP_BITS (1)
#define PICUBED_UART_PARITY (UART_PARITY_NONE)

// Sentinel bytes for commands
// IMPORTANT: Keep in sync with adcs_driver.c on the picubed
#define ADCS_SEND_TELEM ('T')
#define ADCS_HEALTH_CHECK ('?')
#define ADCS_HEALTH_CHECK_SUCCESS ('!')

#define ACK_BYTE '!'
#define SYN_BYTE '$'
#define PACKET_CORRUPTED_BYTE '#'

// Backdoor for flash programming to enable potential OTA in the future
#define ADCS_FLASH_ERASE ('E')
#define ADCS_FLASH_PROGRAM ('P')

// Timeout between bytes in microseconds
// currently set to 1 second
#define ADCS_BYTE_TIMEOUT_US (1000000)

#define INTERRUPT_BYTE_TIMEOUT_US (100000) // 0.1 second.

#define NUM_CRC_BYTES (4) //for CRC32





static slate_t *slate_for_irq; // Need to save to be accessible to IRQ

unsigned int crc32(const uint8_t *message, uint8_t len);

static void reset_parser(){
    slate_for_irq -> interrupt_state = REST; // in case we timed out, make the state equal to rest, and reset all variables.
    slate_for_irq->temp_queue_iter = 0;
    slate_for_irq->num_data_bytes = 0;
    slate_for_irq->num_data_bytes_received = 0;
    slate_for_irq->num_crc_bytes_received = 0;
}

// RX interrupt handler
static void uart_rx_callback()
{
    uint8_t ch = uart_getc(SAMWISE_ADCS_PICUBED_UART);
    LOG_DEBUG("[Picubed UART Handler] Received char: %x ; current state is %d", ch, slate_for_irq->interrupt_state);
    if(ch == SYN_BYTE || slate_for_irq->interrupt_state == REST ||
    absolute_time_diff_us(slate_for_irq->picubed_uart_last_received_time, get_absolute_time()) > INTERRUPT_BYTE_TIMEOUT_US){
        reset_parser();
        if(ch == SYN_BYTE){
            slate_for_irq -> interrupt_state = SYNC_RECEIVED;
        }
    } else if (slate_for_irq->interrupt_state == SYNC_RECEIVED){
        //we have just received the length byte
        slate_for_irq->num_data_bytes = ch;
        slate_for_irq->num_data_bytes_received = 0;

        //add the received length to the array.
        slate_for_irq->picubed_uart_queue[slate_for_irq->temp_queue_iter] = ch;
        slate_for_irq->temp_queue_iter++;

        slate_for_irq->interrupt_state = LENGTH_RECEIVED;

        slate_for_irq->temporaryPacket.packet_length = ch;
    } else if (slate_for_irq->interrupt_state == LENGTH_RECEIVED){
        // add the command to the array.
        slate_for_irq->picubed_uart_queue[slate_for_irq->temp_queue_iter] = ch;
        slate_for_irq->temp_queue_iter++; 


        slate_for_irq->interrupt_state = COMMAND_RECEIVED;

        if(slate_for_irq -> num_data_bytes == 0){
            // if we don't have any data bytes being sent, then directly skip to the data finished step.
            slate_for_irq -> interrupt_state = DATA_FINISHED;
        }

        adcs_tx_command currentCommand = (adcs_tx_command) ch;
        (slate_for_irq->temporaryPacket).command = currentCommand; // update the temporary packet we are constructing
    } else if (slate_for_irq->interrupt_state == COMMAND_RECEIVED){
         // add the data byte to the array.
        slate_for_irq->picubed_uart_queue[slate_for_irq->temp_queue_iter] = ch;
        slate_for_irq->temp_queue_iter++; 
        if(slate_for_irq->temp_queue_iter >= 255){ // the max number of packets
            reset_parser();
            slate_for_irq->interrupt_state = REST;
        }
        

        slate_for_irq->temporaryPacket.packet_data[slate_for_irq->num_data_bytes_received] = ch; // update the temporary packet we are constructing
        slate_for_irq->num_data_bytes_received += 1;
        if(slate_for_irq->num_data_bytes_received == slate_for_irq->num_data_bytes){
            // then we have finished putting in all the bytes
            slate_for_irq -> interrupt_state = DATA_FINISHED;
            slate_for_irq -> num_crc_bytes_received = 0;
        }
    } else if (slate_for_irq->interrupt_state == DATA_FINISHED){
        //that means we receive a CRC32 packet here.
        slate_for_irq->crc32_value[slate_for_irq->num_crc_bytes_received] = ch;
        slate_for_irq->num_crc_bytes_received += 1;
        if(slate_for_irq->num_crc_bytes_received == NUM_CRC_BYTES){
            
            //now check if the crc matches up!
            unsigned int crcresult = crc32(slate_for_irq->picubed_uart_queue, (slate_for_irq->num_data_bytes) + 2);
            if(memcmp(slate_for_irq->crc32_value, &crcresult, 4) == 0){
                // SEND ACKNOWLEDGEMENT!
                //now, copy the temporary packet we constructed into the queue.
                if(!queue_try_add(&slate_for_irq->picubed_execution_queue, &(slate_for_irq->temporaryPacket))){
                    uart_putc_raw(SAMWISE_ADCS_PICUBED_UART, PACKET_CORRUPTED_BYTE);
                } else {
                    uart_putc_raw(SAMWISE_ADCS_PICUBED_UART, ACK_BYTE);
                }
                

            } else {
                // SEND ERROR!
                uart_putc_raw(SAMWISE_ADCS_PICUBED_UART, PACKET_CORRUPTED_BYTE);
            }
            slate_for_irq->interrupt_state = REST;
        }
    } 

    slate_for_irq->picubed_uart_last_received_time = get_absolute_time();
}

unsigned int crc32(const uint8_t *message, uint8_t len)
{
    size_t i;
    unsigned int byte, crc, mask;

    i = 0;
    crc = 0xFFFFFFFF;
    while (i < len)
    {
        byte = message[i]; // Get next byte.
        crc = crc ^ byte;
        for (int j = 0; j < 8; j++)
        { // Do eight times.
            mask = -(crc & 1);
            crc = (crc >> 1) ^ (0xEDB88320 & mask);
        }
        i = i + 1;
    }
    return ~crc;
}


/**
 * @brief Helper function to read up to num_bytes bytes from ADCS uart with a
 * timeout in the case of missing bytes
 *
 * @param buf           Buffer to read into
 * @param num_bytes     Maximum number of bytes to read
 * @param timeout_us    Timeout between bytes in microseconds
 *
 * @return Number of bytes reac successfully (between 0 and num_bytes inclusive)
 */
static uint32_t read_uart_with_timeout(char *buf, uint32_t num_bytes,
                                       uint32_t timeout_us)
{
    for (uint32_t i = 0; i < num_bytes; i++)
    {
        if (uart_is_readable_within_us(SAMWISE_ADCS_PICUBED_UART, timeout_us))
        {
            buf[i] = uart_getc(SAMWISE_ADCS_PICUBED_UART);
        }
        else
        {
            return i;
        }
    }

    return num_bytes;
}

/**
 * Send a telemetry packet to the picubed over uart
 */
static void send_packet(const adcs_packet_t *packet)
{
    // log current telemetry packet
    LOG_DEBUG(
        "[picubed-uart] Sending telemetry packet: w=%.2f, q0=%.2f, q1=%.2f, "
        "q2=%.2f, q3=%.2f, state=%c, boot_count=%u",
        packet->w, packet->q0, packet->q1, packet->q2, packet->q3,
        packet->state, packet->boot_count);

    const char *data = (const char *)packet;

    // Log *data
    LOG_DEBUG("[picubed-uart] Sending telemetry packet data: %u",
              sizeof(adcs_packet_t));

    // Send bytes one by one
    for (uint32_t i = 0; i < sizeof(adcs_packet_t); i++)
    {
        printf("%02x ", (unsigned char)data[i]);
        uart_putc_raw(SAMWISE_ADCS_PICUBED_UART, data[i]);
    }
    printf("\n");
}

/**
 * Handle a command byte from the picubed.
 *
 * @param slate
 * @param cmd       Command byte to handle
 * @return True on success, false otherwise.
 */
static bool handle_command_packet(slate_t *slate, adcs_command_packet pck)
{
    //TODO: work on this!!
    char command = pck.command;
    
    
    // get the command
    // sleep_ms(50); // Sleep for 20 milliseconds to simulate work

    switch (command)
    {
        case ADCS_SEND_TELEM:
            // Send telemetry
            send_packet(&slate->telemetry);
            return true;
        case ADCS_HEALTH_CHECK:
            // Send a sentinel byte to report as healthy
            uart_putc_raw(SAMWISE_ADCS_PICUBED_UART, ADCS_HEALTH_CHECK_SUCCESS);
            return true;

            // *********************************************************************
            // The following section of code exists purely to provide a backdoor
            // for potential future OTA-ing of the ADCS board.
            //
            // DANGER: USING THIS INTERFACE INCORRECTLY COULD BRICK THE BOARD
            // PERMANENTLY. ONLY COMMAND THIS IF YOU ARE 100% SURE THAT IT WILL
            // WORK. IDEALLY THAT FIRST THING THAT EVER GETS PROGRAMMED HERE IS
            // A LESS DUMB OTA INTERFACE.

        case ADCS_FLASH_ERASE:
        {
            // Erase a single flash sector at the provided offset
            uint32_t offset;
            uint32_t num_bytes = read_uart_with_timeout(
                (char *)&offset, sizeof(offset), ADCS_BYTE_TIMEOUT_US);

            if (num_bytes < sizeof(offset))
            {
                LOG_ERROR(
                    "[picubed-uart] Bad flash erase command! Got %d/4 bytes",
                    num_bytes);
                return false;
            }

            LOG_INFO("[picubed-uart] Erasing flash offset 0x%x", offset);
            flash_range_erase(offset, 1);
            return true;
        }
        case ADCS_FLASH_PROGRAM:
        {
            // Program 256 bytes into flash at the desired offset
            uint32_t offset;
            uint32_t num_bytes = read_uart_with_timeout(
                (char *)offset, sizeof(offset), ADCS_BYTE_TIMEOUT_US);

            if (num_bytes < sizeof(offset))
            {
                LOG_ERROR(
                    "[picubed-uart] Bad flash program command! Got %d/4 bytes",
                    num_bytes);
                return false;
            }

            uint8_t data[256];
            num_bytes = read_uart_with_timeout((char *)data, sizeof(data),
                                               ADCS_BYTE_TIMEOUT_US);

            if (num_bytes < sizeof(offset))
                if (num_bytes < sizeof(data))
                {
                    LOG_ERROR("[picubed-uart] Bad flash program command! Got "
                              "%d/256 bytes",
                              num_bytes);
                    return false;
                }

            LOG_INFO("[picubed-uart] Programing flast at offset 0x%x", offset);
            flash_range_program(offset, data, sizeof(data));
            return true;
        }
            // *********************************************************************

        default:
            // Invalid packet
            LOG_ERROR(
                "[picubed-uart] Encountered an invalid command byte: %c (0x%x)",
                command, command);
            return false;
    }
}

/**
 * Initialize uart for communication with picubed
 *
 */
void picubed_uart_init(slate_t* slate)
{
    slate_for_irq = slate;
    // Initialize uart hardware
    uart_init(SAMWISE_ADCS_PICUBED_UART, PICUBED_UART_BAUD);

    gpio_init(SAMWISE_ADCS_TX_TO_PICUBED);
    gpio_set_function(SAMWISE_ADCS_TX_TO_PICUBED,
                      UART_FUNCSEL_NUM(SAMWISE_ADCS_PICUBED_UART,
                                       SAMWISE_ADCS_TX_TO_PICUBED));

    gpio_init(SAMWISE_ADCS_RX_FROM_PICUBED);
    gpio_set_function(SAMWISE_ADCS_RX_FROM_PICUBED,
                      UART_FUNCSEL_NUM(SAMWISE_ADCS_PICUBED_UART,
                                       SAMWISE_ADCS_RX_FROM_PICUBED));

    // Set data format
    uart_set_format(SAMWISE_ADCS_PICUBED_UART, PICUBED_UART_DATA_BITS,
                    PICUBED_UART_STOP_BITS, PICUBED_UART_PARITY);


    // Set up a RX interrupt
    int UART_IRQ;
    if(SAMWISE_ADCS_PICUBED_UART == uart1){
        UART_IRQ = 21;
    } else {
        UART_IRQ = 20;//??
    }
    UART_IRQ = 37;
    queue_init(&slate->picubed_execution_queue,
               sizeof(adcs_command_packet), /* Size of each element */
               256 /* Max elements */);

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, uart_rx_callback);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(SAMWISE_ADCS_PICUBED_UART, true, false);
}

/**
 * Handle all available command packets from the picubed
 *
 * @return True on success, false otherwise
 */
bool picubed_uart_handle_commands(slate_t *slate) // modify this.
{
    bool all_commands_succeeded = true;


    //shouldn't necessarily drain the full queue; 
    while (!queue_is_empty(&slate->picubed_execution_queue))
    {
        // Read byte and handle it
        adcs_command_packet pck;
        queue_try_remove(&slate->picubed_execution_queue, &pck);
        all_commands_succeeded &= handle_command_packet(slate, pck);
    }

    return all_commands_succeeded;
}
