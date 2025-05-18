// #include "macros.h"
// #include "pico/printf.h"
// #include "pico/stdlib.h"
// #include "pins.h"

// #include "gnc/attitude_filter.h"
// #include "gnc/bdot.h"
// #include "gnc/matrix_utils.h"
// #include "gnc/reaction_wheel_allocation.h"
// #include "gnc/sun_vector.h"

// #include "hardware/i2c.h"

// #include "linalg.h"

#include "drivers/imu.h"
#include "init.h"
#include "macros.h"
#include "slate.h"

#include "pins.h"

slate_t slate;

// #define TEST

static bool reserved_addr(uint8_t addr)
{
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

// void i2c_scan_init()
// {
//     gpio_set_function(SAMWISE_ADCS_I2C0_SCL, GPIO_FUNC_I2C);
//     gpio_set_function(SAMWISE_ADCS_I2C0_SDA, GPIO_FUNC_I2C);
//     gpio_pull_up(SAMWISE_ADCS_I2C0_SCL);
//     gpio_pull_up(SAMWISE_ADCS_I2C0_SDA);

//     gpio_set_function(SAMWISE_ADCS_I2C1_SCL, GPIO_FUNC_I2C);
//     gpio_set_function(SAMWISE_ADCS_I2C1_SDA, GPIO_FUNC_I2C);
//     gpio_pull_up(SAMWISE_ADCS_I2C1_SCL);
//     gpio_pull_up(SAMWISE_ADCS_I2C1_SDA);

//     i2c_init(SAMWISE_ADCS_I2C0, 100000);
//     i2c_init(SAMWISE_ADCS_I2C1, 100000);
// }

void i2c_scan()
{

    LOG_INFO("I2C Bus Scan: I2C0");
    LOG_INFO("\n0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr)
    {
        if (addr % 16 == 0)
        {
            printf("%02x ", addr);
        }

        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_timeout_us(i2c0, addr, &rxdata, 1, false, 1000);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }

    LOG_INFO("I2C Bus Scan: I2C1");
    LOG_INFO("\n0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr)
    {
        if (addr % 16 == 0)
        {
            printf("%02x ", addr);
        }

        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_timeout_us(i2c1, addr, &rxdata, 1, false, 1000);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
}

int main()
{
    //     gpio_init(PICO_DEFAULT_LED_PIN);
    //     gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    //     gpio_put(PICO_DEFAULT_LED_PIN, 1);
    //     sleep_ms(1000);
    //     gpio_put(PICO_DEFAULT_LED_PIN, 0);

    // #ifdef TEST

    //     // Test initialization
    //     attitude_filter_init(&slate);

    // i2c_scan_init();

    // while (1)
    // {
    //     printf("ADCS board is alive :)\n");
    //     sleep_ms(1000);

    //     i2c_scan();

    // // Test bdot control
    // test_bdot_control(&slate);

    // Test attitude propagation
    // test_attitude_filter(&slate);

    // // Test sun vector
    // test_sun_vector_eci(&slate);

    // // Test reaction wheel allocation
    // test_reaction_wheel_allocation();
    // test_matrix_utils();
    // sleep_ms(1000);
    // }
    // #else
    // #endif

    //     while (1)
    //         ;

    init(&slate);
    sleep_ms(5000);

    LOG_INFO("Initialization complete!");
    LOG_INFO("Slate uses %d bytes of memory", sizeof(slate));

    while (1)
    {
        imu_get_rotation(&slate);
    }
}