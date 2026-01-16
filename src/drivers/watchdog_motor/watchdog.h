#pragma once

#include "pico/time.h"
#include "hardware/gpio.h"

typedef struct
{
    uint pin;
    absolute_time_t last_transition;
    bool set;
    uint64_t us_low;
    uint64_t us_high;
} watchdog_t;

watchdog_t watchdog_mk(uint32_t pin);
void watchdog_init(watchdog_t *wd);
void watchdog_feed(watchdog_t *wd);
