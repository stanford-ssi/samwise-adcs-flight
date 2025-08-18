/**
 * @author Lundeen Cahilly and Marc Aaron Reyes
 * @date 2025-08-02
 *
 * ADM1176 power monitor driver
 */

#ifndef ADM_H
#define ADM_H

#include "slate.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize ADM1176 power monitor
 * @return true if initialization successful, false otherwise
 */
bool adm_init(void);

/**
 * Turn on ADM1176 power monitoring
 * @return true if successful, false otherwise
 */
bool adm_power_on(void);

/**
 * Turn off ADM1176 power monitoring
 * @return true if successful, false otherwise
 */
bool adm_power_off(void);

/**
 * Get voltage measurement in volts
 * @param voltage_out Pointer to store voltage value
 * @return true if successful, false otherwise
 */
bool adm_get_voltage(float *voltage_out);

/**
 * Get current measurement in amps
 * @param current_out Pointer to store current value
 * @return true if successful, false otherwise
 */
bool adm_get_current(float *current_out);

/**
 * Read ADM1176 status register
 * @param status_out Pointer to store status value
 * @return true if successful, false otherwise
 */
bool adm_get_status(uint8_t *status_out);

/**
 * Check if ADM1176 driver is initialized
 * @return true if initialized, false otherwise
 */
bool adm_is_initialized(void);

/**
 * Get power consumption in watts and store in slate
 * @param slate Pointer to the slate structure
 * @return true if successful, false otherwise
 */
bool adm_get_power(slate_t *slate);

#endif // ADM_H