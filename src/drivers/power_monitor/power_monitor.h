/**
 * @author Lundeen Cahilly and Marc Aaron Reyes
 * @date 2025-08-02
 *
 * ADM1176 power monitor driver
 */

#include "slate.h"
#include <stdbool.h>
#include <stdint.h>

bool adm_init(void);
bool adm_power_on(void);
bool adm_power_off(void);

// Status functions
bool adm_get_status(uint8_t *status_out);
bool adm_is_initialized(void);

// Power measurement
bool adm_get_power(slate_t *slate);
bool adm_get_voltage(float *voltage_out);
bool adm_get_current(float *current_out);

#ifdef TEST
void init_power_monitor(void);
void read_power_monitor(void);
#endif