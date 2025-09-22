/**
 * @author  Lundeen Cahilly
 * @date    2025-09-22
 *
 * Test functions for MRAM driver validation
 */

#include "hardware/flash.h"
#include "pico/flash.h"
#include "pico/stdlib.h"

#include "constants.h"
#include "macros.h"
#include "pins.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#include <cstring>
#include <stdint.h>
#include <stdlib.h>

void mram_test(void);
void mram_collision_test(void);