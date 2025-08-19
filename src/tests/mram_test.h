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