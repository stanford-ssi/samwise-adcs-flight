#include "apps/adcs_new/slate.h"

#include "gnc/mekf/filter.h"

extern slate_t slate;

void vTaskSensorFusion(void *);

void vTaskPropagate(void *);

void vTaskResetEstimate(void *);


