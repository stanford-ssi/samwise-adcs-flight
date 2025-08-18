#include "mram_test.h"
#include "./drivers/mram.h"

void mram_test(void)
{
    LOG_INFO("[mram_test] Starting MRAM test...");

    mram_init();
    LOG_INFO("[mram_test] MRAM initialized");

    uint8_t status = mram_read_status();
    LOG_INFO("[mram_test] Status register: 0x%02X", status);

    uint8_t test_data = 0xA5;
    uint8_t read_data = 0x00;

    mram_write(0x000000, &test_data, 1);
    sleep_ms(1); // Add small delay
    mram_read(0x000000, &read_data, 1);

    if (test_data == read_data)
    {
        LOG_INFO("[mram_test] PASS - Wrote 0x%02X, read 0x%02X", test_data,
                 read_data);
    }
    else
    {
        LOG_ERROR("[mram_test] FAIL - Wrote 0x%02X, read 0x%02X", test_data,
                  read_data);
    }
}