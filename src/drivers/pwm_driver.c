#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "pins.h"
#include "macros.h"
#include "linalg.h"
#include "pwm_driver.h"
using namespace linalg::aliases;

// Prototypes for the functions we will implement
uint8_t init_pwm(i2c_t i2c0) {
    gpio_set_function(SAMWISE_ADCS_X_MAGDRV_IN1, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_X_MAGDRV_IN2, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Y_MAGDRV_IN1, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Y_MAGDRV_IN2, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Z_MAGDRV_IN1, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Z_MAGDRV_IN2, GPIO_FUNC_PWM);

    // Define slices
    uint slice_x1 = pwm_gpio_to_slice_num(SAMWISE_ADCS_X_MAGDRV_IN1);  
    uint slice_x2 = pwm_gpio_to_slice_num(SAMWISE_ADCS_X_MAGDRV_IN2);
    uint slice_y1 = pwm_gpio_to_slice_num(SAMWISE_ADCS_Y_MAGDRV_IN1);
    uint slice_y2 = pwm_gpio_to_slice_num(SAMWISE_ADCS_Y_MAGDRV_IN2);
    uint slice_z1 = pwm_gpio_to_slice_num(SAMWISE_ADCS_Z_MAGDRV_IN1);
    uint slice_z2 = pwm_gpio_to_slice_num(SAMWISE_ADCS_Z_MAGDRV_IN2);
    uint slices[6] = {slice_x1, slice_x2, slice_y1, slice_y2, slice_z1, slice_z2};

    // Calculate PWM parameters
    uint32_t clk_sys = clock_get_hz(clk_sys);
    uint32_t pwm_freq = PWM_FREQUENCY_HZ;
    uint32_t cycles_per_period = clk_sys / pwm_freq; // cycles per period
    uint16_t divider = 125; // 125 MHz ÷ (125 × wrap) = 1 MHz base, moderate and good for most applications
    uint16_t wrap = 999; // 1000 cycles per period

    // Set up PWM slices
    for (uint i = 0; i < 6; i++) {
        pwm_set_wrap(slices[i], wrap);
        pwm_set_clkdiv_int_frac(slices[i], divider);
        pwm_set_chan_level(slices[i], PWM_CHAN_A, 0); // Set initial level to 0
        pwm_set_chan_level(slices[i], PWM_CHAN_B, 0); // Set initial level to 0
        pwm_set_enabled(slices[i], true);
    }

    pwm_intialized = true; 
    return 0; // Return 0 to indicate successful initialization
}

uint8_t do_pwm(int8_t xdn, int8_t ydn, int8_t zdn) {
    if (pwm_initialized == false) {
        return 1; // Return 1 to indicate that PWM is not initialized
    }

    // Clamp input values to valid range (-100 to +100)
    xdn = (xdn > 100) ? 100 : (xdn < -100) ? -100 : xdn;
    ydn = (ydn > 100) ? 100 : (ydn < -100) ? -100 : ydn;
    zdn = (zdn > 100) ? 100 : (zdn < -100) ? -100 : zdn;

    // Convert percentage to PWM level
    auto percentage_to_pwm_level = [](int8_t percentage) -> uint16_t {
        uint16_t abs_percentage = (percentage < 0) ? -percentage : percentage;
        return (abs_percentage * pwm_wrap_value) / 100;
    };


}



// basic c code from here, not part of the driver

int main() {
    /// \tag::setup_pwm[]

    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(8, GPIO_FUNC_PWM);
    gpio_set_function(9, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num = pwm_gpio_to_slice_num(8);

    // Set period of 4 cycles (0 to 3 inclusive),
    // which corresponds to a frequency of 125MHz / 4 = 31.25 MHz
    // since the ineternal clock is 125MHz.
    pwm_set_wrap(slice_num, 3);
    // Set channel A output high for one cycle before dropping
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 1);
    // Set initial B output high for three cycles before dropping
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 3);
    // Set the PWM running
    pwm_set_enabled(slice_num, true);
    /// \end::setup_pwm[]

    // Note we could also use pwm_set_gpio_level(gpio, x) which looks up the
    // correct slice and channel for a given GPIO.
}
