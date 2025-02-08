#include "pico/printf.h"
#include "pico/stdlib.h"

#include "gnc/bdot.h"

#include "linalg.h"

using namespace linalg;
using namespace linalg::aliases;

slate_t slate;

#define TEST

int main()
{
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(1000);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

#ifdef TEST
    while (1)
    {
        test_bdot_control(&slate);

        sleep_ms(5000);
    }
#else
#endif

    while (1)
        ;
}