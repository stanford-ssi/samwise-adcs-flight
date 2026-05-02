/**
 * @author Chen Li and Lundeen Cahilly
 * @date 2025-07-04
 */

#pragma once

#include "constants.h"
#include "linalg.h"
#include "drivers/gps/gps.h"

typedef struct sun_vector {
    float3 sun_vector_eci;
} sun_vector_t;

void compute_sun_vector_eci(const gps_data_processed_t &gps_data, 
        sun_vector_t &sun_vector);

#ifdef TEST
void test_sun_vector_eci(gps_data_processed_t &gps_data, sun_vector_t &sun_vector);
void test_sun_vector_year(gps_data_processed_t &gps_data, sun_vector_t &sun_vector);
#endif
