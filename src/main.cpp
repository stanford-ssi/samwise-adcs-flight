#include "pico/printf.h"
#include "pico/stdlib.h"

#include "gnc/attitude_dynamics.h"
#include "gnc/bdot.h"
#include "gnc/matrix_utils.h"
#include "gnc/reaction_wheel_allocation.h"
#include "gnc/sun_vector.h"
#include "gnc/world/b_field.h"

#include "linalg.h"

using namespace linalg;
using namespace linalg::aliases;

slate_t slate;

#define TEST

int main()
{
    stdio_init_all();
    
    /*
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(1000);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    */

    sleep_ms(5000);
    test_compute_B(&slate);
    //test_map_compute_B(&slate);
    //test_legendre_polynomials();

    //test_compute_B(&slate);

    // #ifdef TEST
    //     while (1)
    //     {
    //         // test_bdot_control(&slate);
    //         // test_compute_B(&slate);
    //         test_legendre_polynomials();
    //         // // Test bdot control
    //         // test_bdot_control(&slate);

    //         // // Test attitude propagation
    //         // test_propagate_attitude(&slate);

    //         // // Test sun vector
    //         // test_sun_vector_eci(&slate);

    //         // // Test reaction wheel allocation
    //         // test_reaction_wheel_allocation();
    //         // test_matrix_utils();
    //         sleep_ms(1000);
    //     }
    // #else
    // #endif

    //     while (1)
    //         ;
}
