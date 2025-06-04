#pragma once

#include "adcs_packet.h"
#include "slate.h"

# prototypes
uint8_t adc_init(slate_t *slate);
uint16_t adc_read(slate_t *slate, int16_t *adc_values);


