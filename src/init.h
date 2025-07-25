/**
 * @file init.h
 * @author Niklas Vainio
 * @date 2025-05-08
 *
 * This file defines the main function for initializing hardware on the board
 */

#include "slate.h"

#pragma once

/**
 * @brief Initialize the state of the slate, and all hardware on the board
 *
 * @param slate
 */
void init(slate_t *slate);