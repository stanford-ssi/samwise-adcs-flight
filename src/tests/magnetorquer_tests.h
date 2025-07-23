/**
 * @author  Lundeen Cahilly
 * @date    2025-07-16
 *
 * Magnetorquer test functions
 */

#pragma once

#include <stdbool.h>

/**
 * Initialize magnetorquer test module
 */
void magnetorquer_tests_init(void);

/**
 * Run magnetorquer test sequence
 *
 * @return true when all tests are complete, false while still running
 */
bool magnetorquer_tests_dispatch(void);