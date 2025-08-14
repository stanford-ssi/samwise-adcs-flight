/**
 * @author Lundeen Cahilly
 * @date 2025-08-13
 *
 * This file feeds the watchdog timer to prevent system resets.
 */

#pragma once

#include "pico/time.h"

void watchdog_init(void);
void watchdog_feed(void);