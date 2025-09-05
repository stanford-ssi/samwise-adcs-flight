/**
 * @author  Lundeen Cahilly
 * @date    2025-08-02
 *
 * Read ADM1176 power monitor
 */

#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

/**
 * @brief Initialize power monitoring and log initial power state
 */
void init_power_monitor();

/**
 * @brief Read power monitor and log current power state
 */
void read_power_monitor();

#endif // POWER_MONITOR_H