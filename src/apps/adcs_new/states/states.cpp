#include "apps/adcs_new/states/states.h"
#include "apps/adcs_new/slate.h"

extern slate_t slate;

void init_state_machine() {
    slate.state_machine.events = 
        xEventGroupCreateStatic(&slate.state_machine.event_group);

    // motor_slate.state_queue_handle = xQueueCreateStatic(SENSOR_QUEUE_DEPTH,
    //         sizeof(StateMsg_t),
    //         state_queue_buffer,
    //         &state_queue);
}

