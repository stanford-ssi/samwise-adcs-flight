#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "apps/adcs_new/states/states.h"
#include "apps/adcs_new/tasks/tasks.h"
#include "apps/adcs_new/slate.h"

extern slate_t slate;

void init_state_machine() {
    slate.state_machine.events = 
        xEventGroupCreateStatic(&slate.state_machine.event_group);

    slate.state_machine.state_queue_handle = xQueueCreateStatic(SENSOR_QUEUE_DEPTH,
            sizeof(StateMsg_t),
            slate.state_machine.state_queue_buffer,
            &slate.state_machine.state_queue);
    
    // Task groups, cheapest to richest.
    //
    // WATCHDOG and TELEMETRY run in every operational state: dropping the
    // watchdog resets the board, and dropping telemetry makes the fault
    // invisible from the ground.
    constexpr EventBits_t housekeeping_tasks
        = TASK_BIT(TASK_WATCHDOG)
        | TASK_BIT(TASK_TELEMETRY);

    // Everything needed to observe attitude, but nothing that acts on it.
    constexpr EventBits_t sensor_tasks
        = TASK_BIT(TASK_GPS)
        | TASK_BIT(TASK_IMU)
        | TASK_BIT(TASK_MAGNETOMETER)
        | TASK_BIT(TASK_SUN_SENSOR);

    // The MEKF. Requires a GPS fix, because both reference vectors are derived
    // from position and time.
    constexpr EventBits_t estimator_tasks
        = TASK_BIT(TASK_PROPAGATE)
        | TASK_BIT(TASK_RESET_ESTIMATE)
        | TASK_BIT(TASK_MAGNETOMETER_FUSION)
        | TASK_BIT(TASK_SUN_VECTOR_FUSION);

    // The only tasks that draw significant power. vTaskActuators additionally
    // refuses to energise the rods unless slate.magtorq_armed is set, so
    // enabling these here is permission to torque, not a guarantee of it.
    constexpr EventBits_t control_tasks
        = TASK_BIT(TASK_BDOT)
        | TASK_BIT(TASK_ACTUATORS);

    slate.state_machine.state_configs[STATE_INIT].enabled_bits
        = TASK_BIT(TASK_INIT);

    // Low bus voltage. Shed everything optional: no estimator, no actuation,
    // and no GPS or sun sensors either. IMU and magnetometer are cheap and give
    // the ground a tumble rate and a field vector to diagnose from.
    slate.state_machine.state_configs[STATE_SAFE].enabled_bits
        = housekeeping_tasks
        | TASK_BIT(TASK_IMU)
        | TASK_BIT(TASK_MAGNETOMETER);

    // Commanded off from the ground.
    slate.state_machine.state_configs[STATE_DISABLED].enabled_bits
        = housekeeping_tasks;

    // Nominal pre-fix mode: all sensors, no estimator (no ECI reference yet),
    // no actuation.
    slate.state_machine.state_configs[STATE_ENABLED].enabled_bits
        = housekeeping_tasks
        | sensor_tasks;

    // Full estimation mode. B-dot and the rods are deliberately absent: only
    // an explicit detumble command may enable control_tasks.
    slate.state_machine.state_configs[STATE_FUSION].enabled_bits
        = housekeeping_tasks
        | sensor_tasks
        | estimator_tasks;

    // Detumbling is never entered autonomously. It is a ground-commanded mode
    // with only the sensors and control tasks required by B-dot; it does not
    // wait for GPS or run the attitude estimator.
    slate.state_machine.state_configs[STATE_DETUMBLE].enabled_bits
        = housekeeping_tasks
        | sensor_tasks
        | control_tasks;
}

void enter_state(StateId_t new_state) {
    if (new_state != STATE_DETUMBLE) {
        // Clear software commands whenever detumbling is not the selected mode.
        // The actuator task independently checks the state and turns PWM off.
        slate.bdot.bdot_has_prev_data_ = false;
        slate.bdot.bdot_data_has_updated_ = false;
        slate.magtorq.magtorq_requested = float3(0.0f, 0.0f, 0.0f);
        slate.magtorq.magtorq_duty_cycle = float3(0.0f, 0.0f, 0.0f);
    }

    switch (new_state) {
        case STATE_SAFE:
            break;
        case STATE_ENABLED:
            break;
        case STATE_DISABLED:
            break;
        case STATE_FUSION:
            for (int i = 0; i < 5; i++) {
                LOG_INFO("////////////////////////");
                LOG_INFO("SWITCHING TO FUSION MODE");
            }
            LOG_INFO("////////////////////////");
            break;
        case STATE_DETUMBLE:
            LOG_INFO("SWITCHING TO COMMANDED DETUMBLE MODE");
            // Never reuse a derivative or actuator request from an earlier
            // detumble session.
            slate.bdot.bdot_has_prev_data_ = false;
            slate.bdot.bdot_data_has_updated_ = false;
            slate.magtorq.magtorq_requested = float3(0.0f, 0.0f, 0.0f);
            slate.magtorq.magtorq_duty_cycle = float3(0.0f, 0.0f, 0.0f);
            break;
        default:
            break;
    };

    EventBits_t wanted = slate.state_machine.state_configs[new_state].enabled_bits;
    EventBits_t all = (1 << NTASKS) - 1; // NTASKS 1s

    // Disable tasks not within state
    xEventGroupClearBits(slate.state_machine.events, all & ~wanted);
    // Publish the state before releasing newly enabled tasks. In particular,
    // the actuator task must observe DETUMBLE before it is allowed to run.
    slate.state_machine.current_state = new_state;
    // Enable tasks within the state
    xEventGroupSetBits(slate.state_machine.events, wanted);
}

void state_handle_message(StateMsg_t msg)
{
    switch(slate.state_machine.current_state) {
        case STATE_DISABLED:
            switch(msg) {
                case MSG_VOLTAGE_LOW:
                    enter_state(STATE_SAFE);
                    break;
                case MSG_COMMAND_ENABLE_NORMAL:
                    enter_state(STATE_ENABLED);
                    break;
                case MSG_COMMAND_ENTER_DETUMBLE:
                    enter_state(STATE_DETUMBLE);
                    break;
                default:
                    break;
            }
            break;
        case STATE_INIT:
            switch(msg) {
                case MSG_INIT_DONE:
                    enter_state(STATE_ENABLED);
                    break;
                default:
                    break;
            }
            break;
        case STATE_SAFE:
            // Safing is entered on low bus voltage, so the ONLY way out is the
            // voltage recovering. Commands and GPS cannot override safing.
            switch(msg) {
                case MSG_VOLTAGE_OK:
                    enter_state(STATE_ENABLED);
                    break;
                default:
                    break;
            }
            break;
        case STATE_ENABLED:
            switch(msg) {
                case MSG_VOLTAGE_LOW:
                    enter_state(STATE_SAFE);
                    break;
                case MSG_GPS_VALID:
                    enter_state(STATE_FUSION);
                    break;
                case MSG_COMMAND_DISABLE:
                    enter_state(STATE_DISABLED);
                    break;
                case MSG_COMMAND_ENTER_DETUMBLE:
                    enter_state(STATE_DETUMBLE);
                    break;
                default:
                    break;
            }
            break;
        case STATE_FUSION:
            switch(msg) {
                case MSG_VOLTAGE_LOW:
                    enter_state(STATE_SAFE);
                    break;
                case MSG_COMMAND_DISABLE:
                    enter_state(STATE_DISABLED);
                    break;
                case MSG_COMMAND_ENTER_DETUMBLE:
                    enter_state(STATE_DETUMBLE);
                    break;
                default:
                    break;
            }
            break;
        case STATE_DETUMBLE:
            switch(msg) {
                case MSG_VOLTAGE_LOW:
                    enter_state(STATE_SAFE);
                    break;
                case MSG_COMMAND_DISABLE:
                    enter_state(STATE_DISABLED);
                    break;
                case MSG_COMMAND_ENABLE_NORMAL:
                    enter_state(STATE_ENABLED);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    } // end switch (state)
}
