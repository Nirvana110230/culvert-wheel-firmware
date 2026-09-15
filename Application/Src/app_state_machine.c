#include "app_state_machine.h"

#include <stddef.h>

static const uint32_t APP_SAFE_STOP_ACTIONS =
    APP_ACTION_CLEAR_ALL_MOTION_COMMANDS |
    APP_ACTION_REQUEST_ZERO_SPEED |
    APP_ACTION_REQUEST_BRAKE_ENGAGE;

static app_transition_result_t result_with_actions(app_state_machine_t machine,
                                                   uint32_t actions)
{
    const app_transition_result_t result = {
        .machine = machine,
        .actions = actions,
    };

    return result;
}

static app_transition_result_t safe_result(app_state_machine_t machine)
{
    return result_with_actions(machine, APP_SAFE_STOP_ACTIONS);
}

static app_transition_result_t safe_cancel_result(app_state_machine_t machine)
{
    return result_with_actions(
        machine, APP_SAFE_STOP_ACTIONS | APP_ACTION_CANCEL_INSPECTION);
}

static app_transition_result_t fault_result(app_state_machine_t machine,
                                            bool cancel_inspection)
{
    machine.state = APP_STATE_FAULT;
    return cancel_inspection ? safe_cancel_result(machine) : safe_result(machine);
}

static bool state_is_valid(app_state_t state)
{
    return (uint32_t)state <= (uint32_t)APP_STATE_FAULT;
}

static bool mode_is_valid(app_mode_selection_t mode)
{
    return (uint32_t)mode <= (uint32_t)APP_MODE_SELECTION_STANDBY;
}

static bool state_has_inspection_activity(app_state_t state)
{
    return (state == APP_STATE_INSPECT_REQUESTED) ||
           (state == APP_STATE_INSPECTING);
}

static bool rc_loss_requires_fault(app_state_t state)
{
    return (state == APP_STATE_BOOT_SAFE) ||
           (state == APP_STATE_REMOTE_ARMING) ||
           (state == APP_STATE_REMOTE) ||
           (state == APP_STATE_STANDBY) ||
           (state == APP_STATE_INSPECT_REQUESTED) ||
           (state == APP_STATE_INSPECTING);
}

app_state_machine_t app_state_machine_initial(void)
{
    const app_state_machine_t machine = {
        .state = APP_STATE_BOOT_SAFE,
        .selected_mode = APP_MODE_SELECTION_INVALID,
    };

    return machine;
}

static app_transition_result_t handle_boot_safe(app_state_machine_t machine,
                                                app_event_t event)
{
    if (event == APP_EVENT_SELF_CHECK_PASSED)
    {
        if (machine.selected_mode == APP_MODE_SELECTION_REMOTE)
        {
            machine.state = APP_STATE_REMOTE_ARMING;
        }
        else if (machine.selected_mode == APP_MODE_SELECTION_STANDBY)
        {
            machine.state = APP_STATE_STANDBY;
        }

        return safe_result(machine);
    }

    return result_with_actions(machine, APP_ACTION_NONE);
}

static app_transition_result_t handle_remote_arming(app_state_machine_t machine,
                                                    app_event_t event)
{
    if ((event == APP_EVENT_RC_NEUTRAL_CONFIRMED) &&
        (machine.selected_mode == APP_MODE_SELECTION_REMOTE))
    {
        machine.state = APP_STATE_REMOTE;
        return result_with_actions(
            machine, APP_ACTION_CLEAR_ALL_MOTION_COMMANDS);
    }

    if (event == APP_EVENT_RC_SELECT_STANDBY)
    {
        machine.state = APP_STATE_STANDBY;
        return safe_result(machine);
    }

    if (event == APP_EVENT_RC_NEUTRAL_CONFIRMED)
    {
        return safe_result(machine);
    }

    return result_with_actions(machine, APP_ACTION_NONE);
}

static app_transition_result_t handle_remote(app_state_machine_t machine,
                                             app_event_t event)
{
    if (event == APP_EVENT_RC_SELECT_STANDBY)
    {
        machine.state = APP_STATE_STANDBY;
        return safe_result(machine);
    }

    return result_with_actions(machine, APP_ACTION_NONE);
}

static app_transition_result_t handle_standby(app_state_machine_t machine,
                                              app_event_t event)
{
    if (event == APP_EVENT_RC_SELECT_REMOTE)
    {
        machine.state = APP_STATE_REMOTE_ARMING;
        return safe_result(machine);
    }

    if (event == APP_EVENT_RC_INSPECT_START)
    {
        machine.state = APP_STATE_INSPECT_REQUESTED;
        return result_with_actions(
            machine, APP_SAFE_STOP_ACTIONS | APP_ACTION_REQUEST_INSPECTION);
    }

    return result_with_actions(machine, APP_ACTION_NONE);
}

static app_transition_result_t handle_inspect_requested(
    app_state_machine_t machine,
    app_event_t event)
{
    if (event == APP_EVENT_JETSON_INSPECT_ACCEPTED)
    {
        machine.state = APP_STATE_INSPECTING;
        return result_with_actions(
            machine, APP_ACTION_CLEAR_ALL_MOTION_COMMANDS);
    }

    if ((event == APP_EVENT_JETSON_INSPECT_REJECTED) ||
        (event == APP_EVENT_JETSON_INSPECT_CANCELLED))
    {
        machine.state = APP_STATE_STANDBY;
        return safe_result(machine);
    }

    if (event == APP_EVENT_INSPECT_REQUEST_TIMEOUT)
    {
        machine.state = APP_STATE_STANDBY;
        return safe_cancel_result(machine);
    }

    if (event == APP_EVENT_RC_SELECT_REMOTE)
    {
        machine.state = APP_STATE_REMOTE_ARMING;
        return safe_cancel_result(machine);
    }

    if (event == APP_EVENT_RC_SELECT_STANDBY)
    {
        machine.state = APP_STATE_STANDBY;
        return safe_cancel_result(machine);
    }

    return result_with_actions(machine, APP_ACTION_NONE);
}

static app_transition_result_t handle_inspecting(app_state_machine_t machine,
                                                 app_event_t event)
{
    if (event == APP_EVENT_JETSON_INSPECT_FINISHED)
    {
        machine.state = APP_STATE_STANDBY;
        return safe_result(machine);
    }

    if (event == APP_EVENT_JETSON_INSPECT_CANCELLED)
    {
        machine.state = APP_STATE_STANDBY;
        return safe_result(machine);
    }

    if ((event == APP_EVENT_JETSON_MOTION_TIMEOUT) ||
        (event == APP_EVENT_RC_SELECT_STANDBY))
    {
        machine.state = APP_STATE_STANDBY;
        return safe_cancel_result(machine);
    }

    if (event == APP_EVENT_RC_SELECT_REMOTE)
    {
        machine.state = APP_STATE_REMOTE_ARMING;
        return safe_cancel_result(machine);
    }

    return result_with_actions(machine, APP_ACTION_NONE);
}

static app_transition_result_t handle_emergency(app_state_machine_t machine,
                                                app_event_t event)
{
    if (event == APP_EVENT_RC_EMERGENCY_RELEASED)
    {
        if (machine.selected_mode == APP_MODE_SELECTION_REMOTE)
        {
            machine.state = APP_STATE_REMOTE_ARMING;
        }
        else if (machine.selected_mode == APP_MODE_SELECTION_STANDBY)
        {
            machine.state = APP_STATE_STANDBY;
        }

        return safe_result(machine);
    }

    return result_with_actions(machine, APP_ACTION_NONE);
}

app_transition_result_t app_state_machine_transition(app_state_machine_t current,
                                                       app_event_t event)
{
    app_transition_result_t result;
    const bool inspection_active = state_has_inspection_activity(current.state);

    if (!state_is_valid(current.state) || !mode_is_valid(current.selected_mode) ||
        ((uint32_t)event >= (uint32_t)APP_EVENT_COUNT))
    {
        return fault_result(current, inspection_active);
    }

    if (current.state == APP_STATE_FAULT)
    {
        return safe_result(current);
    }

    if ((event == APP_EVENT_MOTOR_FAULT) ||
        (event == APP_EVENT_SELF_CHECK_FAILED) ||
        ((event == APP_EVENT_RC_SIGNAL_LOST) &&
         rc_loss_requires_fault(current.state)))
    {
        return fault_result(current, inspection_active);
    }

    if (event == APP_EVENT_RC_EMERGENCY_ASSERTED)
    {
        current.state = APP_STATE_EMERGENCY;
        return inspection_active ? safe_cancel_result(current) :
                                   safe_result(current);
    }

    if (event == APP_EVENT_RC_SELECT_REMOTE)
    {
        current.selected_mode = APP_MODE_SELECTION_REMOTE;
    }
    else if (event == APP_EVENT_RC_SELECT_STANDBY)
    {
        current.selected_mode = APP_MODE_SELECTION_STANDBY;
    }

    switch (current.state)
    {
        case APP_STATE_BOOT_SAFE:
            result = handle_boot_safe(current, event);
            break;
        case APP_STATE_REMOTE_ARMING:
            result = handle_remote_arming(current, event);
            break;
        case APP_STATE_REMOTE:
            result = handle_remote(current, event);
            break;
        case APP_STATE_STANDBY:
            result = handle_standby(current, event);
            break;
        case APP_STATE_INSPECT_REQUESTED:
            result = handle_inspect_requested(current, event);
            break;
        case APP_STATE_INSPECTING:
            result = handle_inspecting(current, event);
            break;
        case APP_STATE_EMERGENCY:
            result = handle_emergency(current, event);
            break;
        case APP_STATE_FAULT:
        default:
            result = fault_result(current, inspection_active);
            break;
    }

    return result;
}

app_motion_permissions_t app_state_machine_get_permissions(
    const app_state_machine_t *machine)
{
    app_motion_permissions_t permissions = {
        .remote_motion_allowed = false,
        .jetson_motion_allowed = false,
    };

    if ((machine == NULL) || !state_is_valid(machine->state))
    {
        return permissions;
    }

    permissions.remote_motion_allowed = (machine->state == APP_STATE_REMOTE);
    permissions.jetson_motion_allowed = (machine->state == APP_STATE_INSPECTING);
    return permissions;
}
