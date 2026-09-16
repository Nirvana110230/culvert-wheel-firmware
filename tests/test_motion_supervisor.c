#include "motion_supervisor.h"

#include <stdbool.h>
#include <stdio.h>

static unsigned tests_run;

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
            return false;                                                       \
        }                                                                       \
    } while (0)

#define RUN(test)                                                               \
    do                                                                          \
    {                                                                           \
        ++tests_run;                                                            \
        if (!(test)())                                                          \
        {                                                                       \
            return 1;                                                           \
        }                                                                       \
    } while (0)

static motion_supervisor_t ready_supervisor(void)
{
    motion_supervisor_t supervisor;
    motion_supervisor_config_t config = motion_supervisor_default_config();

    config.motion_enable = true;
    config.rc_timeout_ms = 100u;
    config.jetson_timeout_ms = 500u;
    motion_supervisor_init(&supervisor, &config);
    motion_supervisor_set_mssd_configuration(&supervisor, true, true);
    motion_supervisor_set_kinematics_ready(&supervisor, true);
    motion_supervisor_set_motor_feedback(&supervisor, true, 0u, true, 0u);
    return supervisor;
}

static bool test_remote_uses_only_fresh_rc(void)
{
    motion_supervisor_t supervisor = ready_supervisor();
    motion_decision_t decision;

    motion_supervisor_set_app_state(&supervisor, APP_STATE_REMOTE,
                                    APP_ACTION_CLEAR_ALL_MOTION_COMMANDS);
    motion_supervisor_submit_jetson(&supervisor, 900.0, 300.0, 0u);
    motion_supervisor_submit_rc(&supervisor, 400.0, -200.0, 1u);
    decision = motion_supervisor_poll(&supervisor, 50u);
    CHECK(decision.control_source == MOTION_CONTROL_RC);
    CHECK(decision.motion_authorized);
    CHECK(decision.linear_mm_s == 400.0);
    CHECK(decision.angular_mrad_s == -200.0);

    decision = motion_supervisor_poll(&supervisor, 102u);
    CHECK(!decision.motion_authorized);
    CHECK(decision.stop_requested);
    CHECK(decision.stop_reason == MOTION_STOP_RC_SIGNAL_LOST);
    return true;
}

static bool test_inspecting_uses_only_fresh_jetson_and_times_out(void)
{
    motion_supervisor_t supervisor = ready_supervisor();
    motion_decision_t decision;

    motion_supervisor_set_app_state(&supervisor, APP_STATE_INSPECTING,
                                    APP_ACTION_CLEAR_ALL_MOTION_COMMANDS);
    motion_supervisor_submit_rc(&supervisor, 100.0, 100.0, 10u);
    motion_supervisor_submit_jetson(&supervisor, 700.0, -300.0, 20u);
    decision = motion_supervisor_poll(&supervisor, 500u);
    CHECK(decision.control_source == MOTION_CONTROL_JETSON);
    CHECK(decision.motion_authorized);
    CHECK(decision.linear_mm_s == 700.0);

    decision = motion_supervisor_poll(&supervisor, 520u);
    CHECK(!decision.motion_authorized);
    CHECK(decision.stop_reason == MOTION_STOP_JETSON_TIMEOUT);
    return true;
}

static bool test_mode_switch_clears_old_commands(void)
{
    motion_supervisor_t supervisor = ready_supervisor();
    motion_decision_t decision;

    motion_supervisor_set_app_state(&supervisor, APP_STATE_INSPECTING, 0u);
    motion_supervisor_submit_jetson(&supervisor, 500.0, 0.0, 10u);
    CHECK(motion_supervisor_poll(&supervisor, 11u).motion_authorized);
    motion_supervisor_set_app_state(&supervisor, APP_STATE_REMOTE_ARMING,
                                    APP_ACTION_CLEAR_ALL_MOTION_COMMANDS);
    motion_supervisor_set_app_state(&supervisor, APP_STATE_REMOTE,
                                    APP_ACTION_CLEAR_ALL_MOTION_COMMANDS);
    decision = motion_supervisor_poll(&supervisor, 12u);
    CHECK(!decision.motion_authorized);
    CHECK(decision.control_source == MOTION_CONTROL_NONE);
    CHECK(decision.stop_requested);
    return true;
}

static bool test_standby_emergency_and_fault_priorities(void)
{
    motion_supervisor_t supervisor = ready_supervisor();
    motion_decision_t decision;

    motion_supervisor_set_app_state(&supervisor, APP_STATE_STANDBY, 0u);
    decision = motion_supervisor_poll(&supervisor, 0u);
    CHECK(decision.stop_requested);
    CHECK(decision.stop_type == MSSD_STOP_NORMAL);

    motion_supervisor_set_app_state(&supervisor, APP_STATE_EMERGENCY, 0u);
    decision = motion_supervisor_poll(&supervisor, 1u);
    CHECK(decision.stop_type == MSSD_STOP_EMERGENCY);
    CHECK(decision.stop_reason == MOTION_STOP_EMERGENCY);

    motion_supervisor_set_app_state(&supervisor, APP_STATE_REMOTE, 0u);
    motion_supervisor_submit_rc(&supervisor, 100.0, 0.0, 2u);
    motion_supervisor_set_motor_feedback(&supervisor, true, 1u, true, 0u);
    decision = motion_supervisor_poll(&supervisor, 3u);
    CHECK(!decision.motion_authorized);
    CHECK(decision.stop_reason == MOTION_STOP_MOTOR_FAULT);
    CHECK(decision.stop_type == MSSD_STOP_EMERGENCY);
    return true;
}

static bool test_configuration_communication_and_partial_commit_lock_motion(void)
{
    motion_supervisor_t supervisor = ready_supervisor();
    motion_decision_t decision;

    motion_supervisor_set_app_state(&supervisor, APP_STATE_REMOTE, 0u);
    motion_supervisor_submit_rc(&supervisor, 100.0, 0.0, 0u);
    motion_supervisor_set_mssd_configuration(&supervisor, true, false);
    decision = motion_supervisor_poll(&supervisor, 1u);
    CHECK(decision.stop_reason == MOTION_STOP_CONFIGURATION);

    motion_supervisor_set_mssd_configuration(&supervisor, true, true);
    motion_supervisor_set_communication_fault(&supervisor, true);
    decision = motion_supervisor_poll(&supervisor, 2u);
    CHECK(decision.stop_reason == MOTION_STOP_COMMUNICATION);

    motion_supervisor_set_communication_fault(&supervisor, false);
    motion_supervisor_record_partial_commit(&supervisor);
    decision = motion_supervisor_poll(&supervisor, 3u);
    CHECK(decision.stop_reason == MOTION_STOP_PARTIAL_COMMIT);
    CHECK(!decision.motion_authorized);
    CHECK(supervisor.partial_commit_count == 1u);
    return true;
}

static bool test_default_lock_and_tick_wrap_are_safe(void)
{
    motion_supervisor_t supervisor;
    const motion_supervisor_config_t config = motion_supervisor_default_config();
    motion_decision_t decision;

    motion_supervisor_init(&supervisor, &config);
    motion_supervisor_set_mssd_configuration(&supervisor, true, true);
    motion_supervisor_set_kinematics_ready(&supervisor, true);
    motion_supervisor_set_motor_feedback(&supervisor, true, 0u, true, 0u);
    motion_supervisor_set_app_state(&supervisor, APP_STATE_REMOTE, 0u);
    motion_supervisor_submit_rc(&supervisor, 100.0, 0.0, UINT32_MAX - 20u);
    decision = motion_supervisor_poll(&supervisor, 10u);
    CHECK(!decision.motion_authorized);
    CHECK((decision.inhibit_reasons & MOTION_INHIBIT_APPLICATION_LOCK) != 0u);
    return true;
}

int main(void)
{
    RUN(test_remote_uses_only_fresh_rc);
    RUN(test_inspecting_uses_only_fresh_jetson_and_times_out);
    RUN(test_mode_switch_clears_old_commands);
    RUN(test_standby_emergency_and_fault_priorities);
    RUN(test_configuration_communication_and_partial_commit_lock_motion);
    RUN(test_default_lock_and_tick_wrap_are_safe);
    printf("all %u motion_supervisor tests passed\n", tests_run);
    return 0;
}
