#include "mram_test.h"
#include "../drivers/mram.h"

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

    // Test clearing a region
    mram_clear(0x000000, 1);
    sleep_ms(1); // Add small delay
    mram_read(0x000000, &read_data, 1);

    if (read_data == 0x00)
    {
        LOG_INFO("[mram_test] PASS - Cleared address 0x000000, read 0x%02X",
                 read_data);
    }
    else
    {
        LOG_ERROR("[mram_test] FAIL - Cleared address 0x000000, read 0x%02X",
                  read_data);
    }
}

void mram_collision_test(void)
{
    LOG_INFO("[mram_test] Starting collision tests...");

    // Reset state by clearing test regions to free any existing allocations
    mram_clear(0x0000, 256); // Clear first 256 bytes
    mram_clear(0x1000, 256); // Clear test region 1
    mram_clear(0x2000, 256); // Clear test region 2
    mram_clear(0x3000, 256); // Clear test region 3

    uint8_t test_data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    bool result;

    // First Write
    LOG_INFO("[mram_test] First Write");
    result = mram_write(0x1000, test_data, 4);
    if (result)
    {
        LOG_INFO("[mram_test] PASS - First write succeeded");
    }
    else
    {
        LOG_ERROR("[mram_test] FAIL - First write failed");
    }

    // Exact Overlap
    LOG_INFO("[mram_test] Exact Overlap");
    result = mram_write(0x1000, test_data, 4);
    if (!result)
    {
        LOG_INFO("[mram_test] PASS - Exact overlap correctly blocked");
    }
    else
    {
        LOG_ERROR("[mram_test] FAIL - Exact overlap was not blocked");
    }

    // Partial Overlap
    LOG_INFO("[mram_test] Partial Overlap");
    result = mram_write(0x1002, test_data, 4);
    if (!result)
    {
        LOG_INFO("[mram_test] PASS - Partial overlap correctly blocked");
    }
    else
    {
        LOG_ERROR("[mram_test] FAIL - Partial overlap was not blocked");
    }

    // Adjacent Write
    LOG_INFO("[mram_test] Adjacent Write");
    result = mram_write(0x1004, test_data, 4);
    if (result)
    {
        LOG_INFO("[mram_test] PASS - Adjacent write succeeded");
    }
    else
    {
        LOG_ERROR("[mram_test] FAIL - Adjacent write was blocked");
    }

    // Clear and Rewrite
    LOG_INFO("[mram_test] Clear and Rewrite");
    mram_clear(0x1000, 8);
    result = mram_write(0x1000, test_data, 4);
    if (result)
    {
        LOG_INFO("[mram_test] PASS - Rewrite after clear succeeded");
    }
    else
    {
        LOG_ERROR("[mram_test] FAIL - Rewrite after clear failed");
    }

    // Multiple Regions
    LOG_INFO("[mram_test] Multiple Regions");
    result = mram_write(0x2000, test_data, 4);
    bool result2 = mram_write(0x3000, test_data, 4);
    bool result3 = mram_write(0x2002, test_data, 4);

    if (result && result2 && !result3)
    {
        LOG_INFO("[mram_test] PASS - Multiple regions: separate succeeded, "
                 "overlap blocked");
    }
    else
    {
        LOG_ERROR("[mram_test] FAIL - Multiple regions test failed");
    }

    LOG_INFO("[mram_test] Automatic collision tests complete");
}