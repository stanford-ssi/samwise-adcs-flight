/**
 * @author Iris Xu
 * @date 2025-06-04
 *
 *
 */

#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>

void init_pwm()
{
    /// \tag::setup_pwm[]
    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(SAMWISE_ADCS_X_MAGDRV_IN1, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_X_MAGDRV_IN2, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Y_MAGDRV_IN1, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Y_MAGDRV_IN2, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Z_MAGDRV_IN1, GPIO_FUNC_PWM);
    gpio_set_function(SAMWISE_ADCS_Z_MAGDRV_IN2, GPIO_FUNC_PWM);
}

uint8_t do_pwm(int8_t xdn, int8_t ydn, int8_t zdn, int max_current)
{
    if (xdn + ydn + zdn > 256)
    {
        printf("Error: exceeding max current");
    }
    // make sure all arguments are between -128 and 128
    if ((xdn > 128 || xdn < -128) || (ydn > 128 || ydn < -128) ||
        (zdn > 128 || zdn < -128))
    {
        printf("Error: argument not in range");
    }
    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num[6] = {0, 0, 0, 0, 0, 0};
    slice_num[0] = pwm_gpio_to_slice_num(SAMWISE_ADCS_X_MAGDRV_IN1);
    slice_num[1] = pwm_gpio_to_slice_num(SAMWISE_ADCS_X_MAGDRV_IN2);
    slice_num[2] = pwm_gpio_to_slice_num(SAMWISE_ADCS_Y_MAGDRV_IN1);
    slice_num[3] = pwm_gpio_to_slice_num(SAMWISE_ADCS_Y_MAGDRV_IN2);
    slice_num[4] = pwm_gpio_to_slice_num(SAMWISE_ADCS_Y_MAGDRV_IN1);
    slice_num[5] = pwm_gpio_to_slice_num(SAMWISE_ADCS_Y_MAGDRV_IN2);
    for (int i = 0; i < 6; i++)
    {
        pwm_set_wrap(slice_num[i / 2],
                     255); // replace max_current with period (# of cycles)
        pwm_set_clkdiv(slice_num[i / 2], 15625);
    }
    if (xdn <= 0)
    {
        pwm_set_chan_level(slice_num[0], PWM_CHAN_A, 0);
        pwm_set_chan_level(slice_num[1], PWM_CHAN_B, abs(xdn));
    }
    else
    {
        pwm_set_chan_level(slice_num[0], PWM_CHAN_A, abs(xdn));
        pwm_set_chan_level(slice_num[1], PWM_CHAN_B, 0);
    }
    if (ydn <= 0)
    {
        pwm_set_chan_level(slice_num[2], PWM_CHAN_A, 0);
        pwm_set_chan_level(slice_num[3], PWM_CHAN_B, abs(ydn));
    }
    else
    {
        pwm_set_chan_level(slice_num[2], PWM_CHAN_A, abs(ydn));
        pwm_set_chan_level(slice_num[3], PWM_CHAN_B, 0);
    }
    if (zdn <= 0)
    {
        pwm_set_chan_level(slice_num[4], PWM_CHAN_A, 0);
        pwm_set_chan_level(slice_num[5], PWM_CHAN_B, abs(zdn));
    }
    else
    {
        pwm_set_chan_level(slice_num[4], PWM_CHAN_A, abs(zdn));
        pwm_set_chan_level(slice_num[5], PWM_CHAN_B, 0);
    }
    for (int i = 0; i < 6; i++)
    {
        // Set period of max_current cycles (0 to 3 inclusive)
        pwm_set_enabled(slice_num[i / 2], true);
    }
}
void main()
{
    init_pwm();
    do_pwm(128, 128, 128, 400);
    while (1)
        ;
}