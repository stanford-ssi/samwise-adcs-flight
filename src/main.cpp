#include "pico/printf.h"
#include "pico/stdlib.h"

#include "linalg.h"

using namespace linalg;
using namespace linalg::aliases;

int main()
{
    float3 x = {1.3423, 8, 3.12387213786};
    float3 y = {4, 5.12387128376, 6};

    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    while (1)
    {
        x = x + y;
        float z = dot(x, y);

        printf("z = %f\n", z);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(250);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(1000);
    }
}