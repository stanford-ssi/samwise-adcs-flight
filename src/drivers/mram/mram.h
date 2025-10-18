/**
 * @author Lundeen Cahilly
 * @date 2025-08-18
 *
 * This file contains functions for interfacing with MR25H40MDF
 * MRAM using QSPI on a RP2350B chip.
 */

#include <cstddef>
#include <stdbool.h>
#include <stdint.h>

// Memory allocation tracking structure
typedef struct
{
    uint32_t address;
    size_t length;
    bool in_use;
} mram_allocation_t;

// MRAM basic functions
void mram_init(void);
uint8_t mram_read_status(void);
void mram_sleep(void);
void mram_wake(void);

// Read/write functions
void mram_read(uint32_t address, uint8_t *data, size_t length);
void mram_clear(uint32_t address, size_t length);
bool mram_write(uint32_t address, const uint8_t *data, size_t length);
void mram_write_enable(void);
void mram_write_disable(void);

// Allocation tracking functions
void mram_allocation_init(void);
bool mram_ranges_overlap(uint32_t addr1, size_t len1, uint32_t addr2,
                         size_t len2);
bool mram_register_allocation(uint32_t address, size_t length);
bool mram_check_collision(uint32_t address, size_t length);
bool mram_free_allocation(uint32_t address);

#ifdef TEST
void mram_test(void);
void mram_collision_test(void);
#endif