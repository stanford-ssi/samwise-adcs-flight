/**
 * @author Lundeen Cahilly
 * @date 2025-08-13
 *
 * This file feeds the watchdog timer to prevent system resets.
 */

#pragma once

#include "slate.h"

bool watchdog_init(slate_t *slate);

void watchdog_pet(slate_t *slate);