#include "motion_supervisor.h"

#include <stddef.h>
#include <string.h>

static bool state_is_valid(app_state_t state)
{
    return (uint32_t)state <= (uint32_t)APP_STATE_FAULT;
}

static bool command_is_fresh(const motion_command_t *command,
                             uint32_t now_ms,
                             uint32_t timeout_ms)
{
    return command->valid && (timeout_ms != 0u) &&
           ((uint32_t)(now_ms - command->timestamp_ms) < timeout_ms);
}

static motion_decision_t stopped_decision(motion_stop_reason_t reason,
                                          mssd_stop_type_t stop_type,
                                          uint32_t inhibit_reasons,
                                          bool rc_fresh,
                                          bool jetson_fresh)
{
    const motion_decision_t decision = {
        .control_source = MOTION_CONTROL_NONE,
        .linear_mm_s = 0.0,
        .angular_mrad_s = 0.0,
        .rc_fresh = rc_fresh,
        .jetson_fresh = jetson_fresh,
        .motion_authorized = false,
        .stop_requested = true,
        .stop_type = stop_type,
        .stop_reason = reason,
        .inhibit_reasons = inhibit_reasons,
    };

    return decision;
}

motion_supervisor_config_t motion_supervisor_default_config(void)
{
    const motion_supervisor_config_t config = {
        .motion_enable = false,
        .rc_timeout_ms = 100u,
        .jetson_timeout_ms = 500u,
    };

    return config;
}

bool motion_supervisor_init(motion_supervisor_t *supervisor,
                            const motion_supervisor_config_t *config)
{
    if ((supervisor == NULL) || (config == NULL) ||
        (config->rc_timeout_ms == 0u) || (config->jetson_timeout_ms == 0u))
    {
        return false;
    }
    (void)memset(supervisor, 0, sizeof(*supervisor));
    supervisor->config = *config;
    supervisor->app_state = APP_STATE_BOOT_SAFE;
    return true;
}

void motion_supervisor_clear_commands(motion_supervisor_t *supervisor)
{
    if (supervisor != NULL)
    {
        (void)memset(&supervisor->rc_command, 0, sizeof(supervisor->rc_command));
        (void)memset(&supervisor->jetson_command, 0,
                     sizeof(supervisor->jetson_command));
    }
}

void motion_supervisor_set_app_state(motion_supervisor_t *supervisor,
                                     app_state_t state,
                                     uint32_t actions)
{
    if (supervisor == NULL)
    {
        return;
    }
    if ((supervisor->app_state != state) ||
        ((actions & APP_ACTION_CLEAR_ALL_MOTION_COMMANDS) != 0u))
    {
        motion_supervisor_clear_commands(supervisor);
    }
    supervisor->app_state = state;
}

static void submit_command(motion_command_t *command,
                           double linear_mm_s,
                           double angular_mrad_s,
                           uint32_t timestamp_ms)
{
    command->linear_mm_s = linear_mm_s;
    command->angular_mrad_s = angular_mrad_s;
    command->timestamp_ms = timestamp_ms;
    command->valid = true;
}

void motion_supervisor_submit_rc(motion_supervisor_t *supervisor,
                                 double linear_mm_s,
                                 double angular_mrad_s,
                                 uint32_t timestamp_ms)
{
    if (supervisor != NULL)
    {
        submit_command(&supervisor->rc_command, linear_mm_s,
                       angular_mrad_s, timestamp_ms);
    }
}

void motion_supervisor_submit_jetson(motion_supervisor_t *supervisor,
                                     double linear_mm_s,
                                     double angular_mrad_s,
                                     uint32_t timestamp_ms)
{
    if (supervisor != NULL)
    {
        submit_command(&supervisor->jetson_command, linear_mm_s,
                       angular_mrad_s, timestamp_ms);
    }
}

void motion_supervisor_set_mssd_configuration(motion_supervisor_t *supervisor,
                                              bool read,
                                              bool matches)
{
    if (supervisor != NULL)
    {
        supervisor->mssd_configuration_read = read;
        supervisor->mssd_configuration_matches = read && matches;
    }
}

void motion_supervisor_set_kinematics_ready(motion_supervisor_t *supervisor,
                                            bool ready)
{
    if (supervisor != NULL)
    {
        supervisor->kinematics_ready = ready;
    }
}

void motion_supervisor_set_motor_feedback(motion_supervisor_t *supervisor,
                                          bool right_valid,
                                          uint16_t right_fault_code,
                                          bool left_valid,
                                          uint16_t left_fault_code)
{
    if (supervisor != NULL)
    {
        supervisor->right_feedback_valid = right_valid;
        supervisor->left_feedback_valid = left_valid;
        supervisor->right_fault_code = right_fault_code;
        supervisor->left_fault_code = left_fault_code;
    }
}

void motion_supervisor_set_communication_fault(motion_supervisor_t *supervisor,
                                               bool faulted)
{
    if (supervisor != NULL)
    {
        supervisor->communication_fault = faulted;
    }
}

void motion_supervisor_record_partial_commit(motion_supervisor_t *supervisor)
{
    if (supervisor != NULL)
    {
        supervisor->partial_commit_latched = true;
        ++supervisor->partial_commit_count;
    }
}

motion_decision_t motion_supervisor_poll(const motion_supervisor_t *supervisor,
                                         uint32_t now_ms)
{
    bool rc_fresh;
    bool jetson_fresh;
    uint32_t inhibit = MOTION_INHIBIT_NONE;
    const motion_command_t *command;
    motion_control_source_t source;
    motion_decision_t decision;

    if (supervisor == NULL)
    {
        return stopped_decision(MOTION_STOP_COMMUNICATION,
                                MSSD_STOP_EMERGENCY,
                                MOTION_INHIBIT_COMMUNICATION,
                                false, false);
    }
    rc_fresh = command_is_fresh(&supervisor->rc_command, now_ms,
                                supervisor->config.rc_timeout_ms);
    jetson_fresh = command_is_fresh(&supervisor->jetson_command, now_ms,
                                    supervisor->config.jetson_timeout_ms);

    if (!state_is_valid(supervisor->app_state) ||
        (supervisor->app_state == APP_STATE_FAULT))
    {
        return stopped_decision(MOTION_STOP_MOTOR_FAULT, MSSD_STOP_EMERGENCY,
                                MOTION_INHIBIT_APP_STATE,
                                rc_fresh, jetson_fresh);
    }
    if (supervisor->app_state == APP_STATE_EMERGENCY)
    {
        return stopped_decision(MOTION_STOP_EMERGENCY, MSSD_STOP_EMERGENCY,
                                MOTION_INHIBIT_APP_STATE,
                                rc_fresh, jetson_fresh);
    }
    if (supervisor->partial_commit_latched)
    {
        return stopped_decision(MOTION_STOP_PARTIAL_COMMIT,
                                MSSD_STOP_EMERGENCY,
                                MOTION_INHIBIT_PARTIAL_COMMIT,
                                rc_fresh, jetson_fresh);
    }
    if (supervisor->communication_fault)
    {
        return stopped_decision(MOTION_STOP_COMMUNICATION,
                                MSSD_STOP_EMERGENCY,
                                MOTION_INHIBIT_COMMUNICATION,
                                rc_fresh, jetson_fresh);
    }
    if (!supervisor->mssd_configuration_read ||
        !supervisor->mssd_configuration_matches)
    {
        return stopped_decision(MOTION_STOP_CONFIGURATION,
                                MSSD_STOP_EMERGENCY,
                                MOTION_INHIBIT_MSSD_CONFIGURATION,
                                rc_fresh, jetson_fresh);
    }
    if (!supervisor->right_feedback_valid || !supervisor->left_feedback_valid)
    {
        return stopped_decision(MOTION_STOP_MOTOR_FAULT,
                                MSSD_STOP_EMERGENCY,
                                MOTION_INHIBIT_MOTOR_FEEDBACK,
                                rc_fresh, jetson_fresh);
    }
    if ((supervisor->right_fault_code != 0u) ||
        (supervisor->left_fault_code != 0u))
    {
        return stopped_decision(MOTION_STOP_MOTOR_FAULT,
                                MSSD_STOP_EMERGENCY,
                                MOTION_INHIBIT_MOTOR_FAULT,
                                rc_fresh, jetson_fresh);
    }
    if (!supervisor->kinematics_ready)
    {
        return stopped_decision(MOTION_STOP_KINEMATICS, MSSD_STOP_EMERGENCY,
                                MOTION_INHIBIT_KINEMATICS,
                                rc_fresh, jetson_fresh);
    }
    if (!supervisor->config.motion_enable)
    {
        return stopped_decision(MOTION_STOP_APPLICATION_LOCK, MSSD_STOP_NORMAL,
                                MOTION_INHIBIT_APPLICATION_LOCK,
                                rc_fresh, jetson_fresh);
    }

    source = MOTION_CONTROL_NONE;
    command = NULL;
    if (supervisor->app_state == APP_STATE_REMOTE)
    {
        if (!rc_fresh)
        {
            return stopped_decision(MOTION_STOP_RC_SIGNAL_LOST,
                                    MSSD_STOP_EMERGENCY,
                                    MOTION_INHIBIT_RC_STALE,
                                    rc_fresh, jetson_fresh);
        }
        source = MOTION_CONTROL_RC;
        command = &supervisor->rc_command;
    }
    else if (supervisor->app_state == APP_STATE_INSPECTING)
    {
        if (!jetson_fresh)
        {
            return stopped_decision(MOTION_STOP_JETSON_TIMEOUT,
                                    MSSD_STOP_EMERGENCY,
                                    MOTION_INHIBIT_JETSON_STALE,
                                    rc_fresh, jetson_fresh);
        }
        source = MOTION_CONTROL_JETSON;
        command = &supervisor->jetson_command;
    }
    else
    {
        const motion_stop_reason_t reason =
            (supervisor->app_state == APP_STATE_STANDBY) ?
                MOTION_STOP_STANDBY : MOTION_STOP_APPLICATION_LOCK;
        return stopped_decision(reason, MSSD_STOP_NORMAL,
                                MOTION_INHIBIT_APP_STATE,
                                rc_fresh, jetson_fresh);
    }

    (void)memset(&decision, 0, sizeof(decision));
    decision.control_source = source;
    decision.linear_mm_s = command->linear_mm_s;
    decision.angular_mrad_s = command->angular_mrad_s;
    decision.rc_fresh = rc_fresh;
    decision.jetson_fresh = jetson_fresh;
    decision.motion_authorized = (inhibit == MOTION_INHIBIT_NONE);
    decision.stop_type = MSSD_STOP_NORMAL;
    decision.stop_reason = MOTION_STOP_NONE;
    decision.inhibit_reasons = inhibit;
    return decision;
}
