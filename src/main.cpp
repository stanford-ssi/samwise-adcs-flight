#include "pico/printf.h"
#include "pico/stdlib.h"

#include "drivers/picubed_uart.h"
#include "gnc/bdot.h"
#include "gnc/reaction_wheel_allocation.h"
#include "gnc/sun_vector.h"

#include "linalg.h"

using namespace linalg;
using namespace linalg::aliases;

slate_t slate;

#define TEST

int main()
{
    // adcs_packet_t dummy_packet = {.w = 0.021f,
    //                               .q0 = 0.999f,
    //                               .q1 = 0.2313f,
    //                               .q2 = 0.1321f,
    //                               .q3 = 0.1389713f,
    //                               .state = 0xc0,
    //                               .boot_count = 0xfedcba98};

    stdio_init_all();
    // picubed_uart_init();

    // // while (1)
    // // {
    // //     picubed_uart_send_packet(&dummy_packet);
    // //     sleep_ms(1000);
    // // }

    // // PICUBED TEST
    // adcs_telemetry_task_init(&slate);

    // while (1)
    // {
    //     adcs_telemetry_task_dispatch(&slate);
    // }

    //     gpio_init(PICO_DEFAULT_LED_PIN);
    //     gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    //     gpio_put(PICO_DEFAULT_LED_PIN, 1);
    //     sleep_ms(1000);
    //     gpio_put(PICO_DEFAULT_LED_PIN, 0);

    // #ifdef TEST
    //     while (1)
    //     {
    //         // Test bdot control
    //         test_bdot_control(&slate);

    //         // Test sun vector
    //         test_sun_vector_eci(&slate);

    //         // Test reaction wheel allocation
    //         test_reaction_wheel_allocation();

    //         sleep_ms(5000);
    //     }
    // #else
    // #endif

    //     while (1)
    //         ;
}