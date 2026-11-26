#ifndef APP_STATE_MACHINE_H
#define APP_STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_STATE_BOOT_SAFE = 0,
    APP_STATE_REMOTE_ARMING,
    APP_STATE_REMOTE,
    APP_STATE_STANDBY,
    APP_STATE_INSPECT_REQUESTED,
    APP_STATE_INSPECTING,
    APP_STATE_EMERGENCY,
    APP_STATE_FAULT
} app_state_t;

typedef enum
{
    APP_MODE_SELECTION_INVALID = 0,
    APP_MODE_SELECTION_REMOTE,
    APP_MODE_SELECTION_STANDBY
} app_mode_selection_t;

typedef enum
{
    APP_EVENT_SELF_CHECK_PASSED = 0,
    APP_EVENT_SELF_CHECK_FAILED,
    APP_EVENT_RC_SELECT_REMOTE,
    APP_EVENT_RC_SELECT_STANDBY,
    APP_EVENT_RC_NEUTRAL_CONFIRMED,
    APP_EVENT_RC_INSPECT_START,
    APP_EVENT_RC_EMERGENCY_ASSERTED,
    APP_EVENT_RC_EMERGENCY_RELEASED,
    APP_EVENT_RC_SIGNAL_LOST,
    APP_EVENT_JETSON_INSPECT_ACCEPTED,
    APP_EVENT_JETSON_INSPECT_REJECTED,
    APP_EVENT_JETSON_INSPECT_FINISHED,
    APP_EVENT_JETSON_INSPECT_CANCELLED,
    APP_EVENT_INSPECT_REQUEST_TIMEOUT,
    APP_EVENT_JETSON_MOTION_TIMEOUT,
    APP_EVENT_MOTOR_FAULT,
    APP_EVENT_COMMUNICATION_FAULT,
    APP_EVENT_RC_CAPTURE_LOST,
    APP_EVENT_WHEEL_FAULT_RECOVERY_IN_STANDBY,
    APP_EVENT_COUNT
} app_event_t;

typedef enum
{
    APP_ACTION_NONE = 0,
    APP_ACTION_CLEAR_ALL_MOTION_COMMANDS = (1u << 0),
    APP_ACTION_REQUEST_ZERO_SPEED = (1u << 1),
    APP_ACTION_REQUEST_BRAKE_ENGAGE = (1u << 2),
    APP_ACTION_REQUEST_INSPECTION = (1u << 3),
    APP_ACTION_CANCEL_INSPECTION = (1u << 4)
} app_action_t;

typedef enum
{
    CHASSIS_HOST_INSPECTION = 0,
    CHASSIS_STANDALONE_RC
} chassis_operating_profile_t;

typedef struct
{
    app_state_t state;
    app_mode_selection_t selected_mode;
    bool boot_interrupted;
    chassis_operating_profile_t profile;
} app_state_machine_t;

typedef struct
{
    bool remote_motion_allowed;
    bool jetson_motion_allowed;
} app_motion_permissions_t;

typedef struct
{
    app_state_machine_t machine;
    uint32_t actions;
} app_transition_result_t;

app_state_machine_t app_state_machine_initial(void);

app_transition_result_t app_state_machine_transition(app_state_machine_t current,
                                                       app_event_t event);

app_motion_permissions_t app_state_machine_get_permissions(
    const app_state_machine_t *machine);

#endif
