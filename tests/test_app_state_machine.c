#include "app_state_machine.h"

#include <stddef.h>
#include <stdio.h>

static int g_failures;
static int g_tests_run;

#define EXPECT_EQ(expected, actual)                                              \
    do                                                                            \
    {                                                                             \
        const long expected_value = (long)(expected);                             \
        const long actual_value = (long)(actual);                                 \
        if (expected_value != actual_value)                                       \
        {                                                                         \
            (void)printf("FAIL %s:%d: expected %ld, got %ld\n",                 \
                         __FILE__, __LINE__, expected_value, actual_value);        \
            g_failures++;                                                         \
        }                                                                         \
    } while (0)

#define EXPECT_ACTION_SET(action, actions) EXPECT_EQ((action), ((actions) & (action)))
#define EXPECT_ACTION_CLEAR(action, actions) EXPECT_EQ(0, ((actions) & (action)))

static app_state_machine_t machine_in(app_state_t state)
{
    app_state_machine_t machine = app_state_machine_initial();
    machine.state = state;
    if ((state == APP_STATE_REMOTE_ARMING) || (state == APP_STATE_REMOTE))
    {
        machine.selected_mode = APP_MODE_SELECTION_REMOTE;
    }
    else if ((state == APP_STATE_STANDBY) ||
             (state == APP_STATE_INSPECT_REQUESTED) ||
             (state == APP_STATE_INSPECTING))
    {
        machine.selected_mode = APP_MODE_SELECTION_STANDBY;
    }
    return machine;
}

static app_transition_result_t transition(app_state_machine_t machine,
                                          app_event_t event)
{
    return app_state_machine_transition(machine, event);
}

static void expect_no_motion_permission(const app_state_machine_t *machine)
{
    const app_motion_permissions_t permissions =
        app_state_machine_get_permissions(machine);

    EXPECT_EQ(0, permissions.remote_motion_allowed);
    EXPECT_EQ(0, permissions.jetson_motion_allowed);
}

static void test_power_on_defaults_to_boot_safe(void)
{
    const app_state_machine_t machine = app_state_machine_initial();

    g_tests_run++;
    EXPECT_EQ(APP_STATE_BOOT_SAFE, machine.state);
    EXPECT_EQ(APP_MODE_SELECTION_INVALID, machine.selected_mode);
    expect_no_motion_permission(&machine);
}

static void test_self_check_failure_enters_latched_fault(void)
{
    app_transition_result_t result;

    g_tests_run++;
    result = transition(app_state_machine_initial(), APP_EVENT_SELF_CHECK_FAILED);
    EXPECT_EQ(APP_STATE_FAULT, result.machine.state);
    EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
    EXPECT_ACTION_SET(APP_ACTION_REQUEST_ZERO_SPEED, result.actions);
    EXPECT_ACTION_SET(APP_ACTION_REQUEST_BRAKE_ENGAGE, result.actions);

    result = transition(result.machine, APP_EVENT_RC_SELECT_REMOTE);
    EXPECT_EQ(APP_STATE_FAULT, result.machine.state);
    expect_no_motion_permission(&result.machine);
}

static void test_boot_remote_selection_requires_arming_and_neutral(void)
{
    app_state_machine_t machine = app_state_machine_initial();
    app_transition_result_t result;

    g_tests_run++;
    machine = transition(machine, APP_EVENT_RC_SELECT_REMOTE).machine;
    result = transition(machine, APP_EVENT_SELF_CHECK_PASSED);
    EXPECT_EQ(APP_STATE_REMOTE_ARMING, result.machine.state);
    EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
    expect_no_motion_permission(&result.machine);

    result = transition(result.machine, APP_EVENT_RC_NEUTRAL_CONFIRMED);
    EXPECT_EQ(APP_STATE_REMOTE, result.machine.state);
    EXPECT_EQ(1, app_state_machine_get_permissions(&result.machine).remote_motion_allowed);
}

static void test_boot_standby_selection_enters_standby(void)
{
    app_state_machine_t machine = app_state_machine_initial();
    app_transition_result_t result;

    g_tests_run++;
    machine = transition(machine, APP_EVENT_RC_SELECT_STANDBY).machine;
    result = transition(machine, APP_EVENT_SELF_CHECK_PASSED);
    EXPECT_EQ(APP_STATE_STANDBY, result.machine.state);
    expect_no_motion_permission(&result.machine);
}

static void test_standby_remote_selection_requires_arming(void)
{
    const app_transition_result_t result = transition(
        machine_in(APP_STATE_STANDBY), APP_EVENT_RC_SELECT_REMOTE);

    g_tests_run++;
    EXPECT_EQ(APP_STATE_REMOTE_ARMING, result.machine.state);
    EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
    expect_no_motion_permission(&result.machine);
}

static void test_remote_arming_can_return_to_standby(void)
{
    const app_transition_result_t result = transition(
        machine_in(APP_STATE_REMOTE_ARMING), APP_EVENT_RC_SELECT_STANDBY);

    g_tests_run++;
    EXPECT_EQ(APP_STATE_STANDBY, result.machine.state);
    EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
    expect_no_motion_permission(&result.machine);
}

static void test_remote_arming_rejects_neutral_when_mode_is_not_remote(void)
{
    app_state_machine_t machine = machine_in(APP_STATE_REMOTE_ARMING);
    app_transition_result_t result;

    g_tests_run++;
    machine.selected_mode = APP_MODE_SELECTION_INVALID;
    result = transition(machine, APP_EVENT_RC_NEUTRAL_CONFIRMED);
    EXPECT_EQ(APP_STATE_REMOTE_ARMING, result.machine.state);
    expect_no_motion_permission(&result.machine);
}

static void test_ch5_only_requests_inspection_from_standby(void)
{
    app_transition_result_t result;

    g_tests_run++;
    result = transition(machine_in(APP_STATE_REMOTE), APP_EVENT_RC_INSPECT_START);
    EXPECT_EQ(APP_STATE_REMOTE, result.machine.state);
    EXPECT_ACTION_CLEAR(APP_ACTION_REQUEST_INSPECTION, result.actions);

    result = transition(machine_in(APP_STATE_STANDBY), APP_EVENT_RC_INSPECT_START);
    EXPECT_EQ(APP_STATE_INSPECT_REQUESTED, result.machine.state);
    EXPECT_ACTION_SET(APP_ACTION_REQUEST_INSPECTION, result.actions);
    EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);

    result = transition(result.machine, APP_EVENT_RC_INSPECT_START);
    EXPECT_EQ(APP_STATE_INSPECT_REQUESTED, result.machine.state);
    EXPECT_ACTION_CLEAR(APP_ACTION_REQUEST_INSPECTION, result.actions);
}

static void test_jetson_acceptance_is_required_for_inspecting(void)
{
    app_transition_result_t result;

    g_tests_run++;
    result = transition(machine_in(APP_STATE_STANDBY),
                        APP_EVENT_JETSON_INSPECT_ACCEPTED);
    EXPECT_EQ(APP_STATE_STANDBY, result.machine.state);

    result = transition(machine_in(APP_STATE_INSPECT_REQUESTED),
                        APP_EVENT_JETSON_INSPECT_ACCEPTED);
    EXPECT_EQ(APP_STATE_INSPECTING, result.machine.state);
    EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
    EXPECT_EQ(1, app_state_machine_get_permissions(&result.machine).jetson_motion_allowed);
}

static void test_inspection_request_reject_timeout_and_cancel_return_to_standby(void)
{
    const app_event_t events[] = {
        APP_EVENT_JETSON_INSPECT_REJECTED,
        APP_EVENT_INSPECT_REQUEST_TIMEOUT,
        APP_EVENT_JETSON_INSPECT_CANCELLED,
    };
    size_t index;

    g_tests_run++;
    for (index = 0; index < (sizeof(events) / sizeof(events[0])); index++)
    {
        const app_transition_result_t result = transition(
            machine_in(APP_STATE_INSPECT_REQUESTED), events[index]);
        EXPECT_EQ(APP_STATE_STANDBY, result.machine.state);
        EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
        expect_no_motion_permission(&result.machine);
    }
}

static void test_inspection_request_remote_selection_cancels_and_arms(void)
{
    const app_transition_result_t result = transition(
        machine_in(APP_STATE_INSPECT_REQUESTED), APP_EVENT_RC_SELECT_REMOTE);

    g_tests_run++;
    EXPECT_EQ(APP_STATE_REMOTE_ARMING, result.machine.state);
    EXPECT_ACTION_SET(APP_ACTION_CANCEL_INSPECTION, result.actions);
    EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
    expect_no_motion_permission(&result.machine);
}

static void test_inspecting_exits_safely_for_all_requested_events(void)
{
    const app_event_t standby_events[] = {
        APP_EVENT_JETSON_INSPECT_FINISHED,
        APP_EVENT_JETSON_INSPECT_CANCELLED,
        APP_EVENT_JETSON_MOTION_TIMEOUT,
        APP_EVENT_RC_SELECT_STANDBY,
    };
    size_t index;

    g_tests_run++;
    for (index = 0; index < (sizeof(standby_events) / sizeof(standby_events[0]));
         index++)
    {
        const app_transition_result_t result = transition(
            machine_in(APP_STATE_INSPECTING), standby_events[index]);
        EXPECT_EQ(APP_STATE_STANDBY, result.machine.state);
        EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
        EXPECT_ACTION_SET(APP_ACTION_REQUEST_ZERO_SPEED, result.actions);
        EXPECT_ACTION_SET(APP_ACTION_REQUEST_BRAKE_ENGAGE, result.actions);
    }

    {
        const app_transition_result_t result = transition(
            machine_in(APP_STATE_INSPECTING), APP_EVENT_RC_SELECT_REMOTE);
        EXPECT_EQ(APP_STATE_REMOTE_ARMING, result.machine.state);
        EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
        EXPECT_ACTION_SET(APP_ACTION_CANCEL_INSPECTION, result.actions);
        expect_no_motion_permission(&result.machine);
    }
}

static void test_emergency_assertion_preempts_every_nonfault_state(void)
{
    const app_state_t states[] = {
        APP_STATE_BOOT_SAFE,
        APP_STATE_REMOTE_ARMING,
        APP_STATE_REMOTE,
        APP_STATE_STANDBY,
        APP_STATE_INSPECT_REQUESTED,
        APP_STATE_INSPECTING,
        APP_STATE_EMERGENCY,
    };
    size_t index;

    g_tests_run++;
    for (index = 0; index < (sizeof(states) / sizeof(states[0])); index++)
    {
        const app_transition_result_t result = transition(
            machine_in(states[index]), APP_EVENT_RC_EMERGENCY_ASSERTED);
        EXPECT_EQ(APP_STATE_EMERGENCY, result.machine.state);
        EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
        EXPECT_ACTION_SET(APP_ACTION_REQUEST_ZERO_SPEED, result.actions);
        EXPECT_ACTION_SET(APP_ACTION_REQUEST_BRAKE_ENGAGE, result.actions);
        expect_no_motion_permission(&result.machine);
    }
}

static void test_emergency_release_never_bypasses_remote_arming(void)
{
    app_state_machine_t machine = machine_in(APP_STATE_EMERGENCY);
    app_transition_result_t result;

    g_tests_run++;
    machine.selected_mode = APP_MODE_SELECTION_REMOTE;
    result = transition(machine, APP_EVENT_RC_EMERGENCY_RELEASED);
    EXPECT_EQ(APP_STATE_REMOTE_ARMING, result.machine.state);
    EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
    expect_no_motion_permission(&result.machine);

    machine.selected_mode = APP_MODE_SELECTION_STANDBY;
    result = transition(machine, APP_EVENT_RC_EMERGENCY_RELEASED);
    EXPECT_EQ(APP_STATE_STANDBY, result.machine.state);

    machine.selected_mode = APP_MODE_SELECTION_INVALID;
    result = transition(machine, APP_EVENT_RC_EMERGENCY_RELEASED);
    EXPECT_EQ(APP_STATE_EMERGENCY, result.machine.state);
}

static void test_rc_signal_loss_faults_each_required_state(void)
{
    const app_state_t states[] = {
        APP_STATE_BOOT_SAFE,
        APP_STATE_REMOTE_ARMING,
        APP_STATE_REMOTE,
        APP_STATE_STANDBY,
        APP_STATE_INSPECT_REQUESTED,
        APP_STATE_INSPECTING,
    };
    size_t index;

    g_tests_run++;
    for (index = 0; index < (sizeof(states) / sizeof(states[0])); index++)
    {
        const app_transition_result_t result = transition(
            machine_in(states[index]), APP_EVENT_RC_SIGNAL_LOST);
        EXPECT_EQ(APP_STATE_FAULT, result.machine.state);
        EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
        EXPECT_ACTION_SET(APP_ACTION_REQUEST_ZERO_SPEED, result.actions);
        EXPECT_ACTION_SET(APP_ACTION_REQUEST_BRAKE_ENGAGE, result.actions);
        expect_no_motion_permission(&result.machine);
    }
}

static void test_motor_fault_enters_fault_from_motion_states(void)
{
    const app_state_t states[] = {APP_STATE_REMOTE, APP_STATE_INSPECTING};
    size_t index;

    g_tests_run++;
    for (index = 0; index < (sizeof(states) / sizeof(states[0])); index++)
    {
        const app_transition_result_t result = transition(
            machine_in(states[index]), APP_EVENT_MOTOR_FAULT);
        EXPECT_EQ(APP_STATE_FAULT, result.machine.state);
        EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
        expect_no_motion_permission(&result.machine);
    }
}

static void test_permissions_are_exclusive_and_derived_only_from_state(void)
{
    const app_state_t states[] = {
        APP_STATE_BOOT_SAFE,
        APP_STATE_REMOTE_ARMING,
        APP_STATE_REMOTE,
        APP_STATE_STANDBY,
        APP_STATE_INSPECT_REQUESTED,
        APP_STATE_INSPECTING,
        APP_STATE_EMERGENCY,
        APP_STATE_FAULT,
    };
    size_t index;

    g_tests_run++;
    for (index = 0; index < (sizeof(states) / sizeof(states[0])); index++)
    {
        const app_state_machine_t machine = machine_in(states[index]);
        const app_motion_permissions_t permissions =
            app_state_machine_get_permissions(&machine);
        EXPECT_EQ(states[index] == APP_STATE_REMOTE,
                  permissions.remote_motion_allowed);
        EXPECT_EQ(states[index] == APP_STATE_INSPECTING,
                  permissions.jetson_motion_allowed);
        EXPECT_EQ(0, permissions.remote_motion_allowed &&
                     permissions.jetson_motion_allowed);
    }

    expect_no_motion_permission(NULL);
}

static void test_every_control_source_switch_clears_motion_commands(void)
{
    typedef struct
    {
        app_state_machine_t machine;
        app_event_t event;
        app_state_t expected_state;
    } switch_case_t;

    app_state_machine_t boot_remote = machine_in(APP_STATE_BOOT_SAFE);
    app_state_machine_t emergency_remote = machine_in(APP_STATE_EMERGENCY);
    switch_case_t cases[] = {
        {boot_remote, APP_EVENT_SELF_CHECK_PASSED, APP_STATE_REMOTE_ARMING},
        {machine_in(APP_STATE_REMOTE_ARMING), APP_EVENT_RC_NEUTRAL_CONFIRMED,
         APP_STATE_REMOTE},
        {machine_in(APP_STATE_REMOTE), APP_EVENT_RC_SELECT_STANDBY,
         APP_STATE_STANDBY},
        {machine_in(APP_STATE_STANDBY), APP_EVENT_RC_SELECT_REMOTE,
         APP_STATE_REMOTE_ARMING},
        {machine_in(APP_STATE_STANDBY), APP_EVENT_RC_INSPECT_START,
         APP_STATE_INSPECT_REQUESTED},
        {machine_in(APP_STATE_INSPECT_REQUESTED),
         APP_EVENT_JETSON_INSPECT_ACCEPTED, APP_STATE_INSPECTING},
        {machine_in(APP_STATE_INSPECT_REQUESTED), APP_EVENT_RC_SELECT_REMOTE,
         APP_STATE_REMOTE_ARMING},
        {machine_in(APP_STATE_INSPECTING), APP_EVENT_RC_SELECT_STANDBY,
         APP_STATE_STANDBY},
        {machine_in(APP_STATE_INSPECTING), APP_EVENT_RC_SELECT_REMOTE,
         APP_STATE_REMOTE_ARMING},
        {emergency_remote, APP_EVENT_RC_EMERGENCY_RELEASED,
         APP_STATE_REMOTE_ARMING},
    };
    size_t index;

    g_tests_run++;
    boot_remote.selected_mode = APP_MODE_SELECTION_REMOTE;
    cases[0].machine = boot_remote;
    emergency_remote.selected_mode = APP_MODE_SELECTION_REMOTE;
    cases[9].machine = emergency_remote;

    for (index = 0; index < (sizeof(cases) / sizeof(cases[0])); index++)
    {
        const app_transition_result_t result = transition(
            cases[index].machine, cases[index].event);
        EXPECT_EQ(cases[index].expected_state, result.machine.state);
        EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
    }
}

static void test_known_irrelevant_events_are_ignored_without_actions(void)
{
    typedef struct
    {
        app_state_t state;
        app_event_t event;
    } ignored_case_t;

    const ignored_case_t cases[] = {
        {APP_STATE_BOOT_SAFE, APP_EVENT_JETSON_INSPECT_FINISHED},
        {APP_STATE_REMOTE_ARMING, APP_EVENT_JETSON_INSPECT_ACCEPTED},
        {APP_STATE_REMOTE, APP_EVENT_RC_INSPECT_START},
        {APP_STATE_STANDBY, APP_EVENT_JETSON_MOTION_TIMEOUT},
        {APP_STATE_INSPECT_REQUESTED, APP_EVENT_JETSON_INSPECT_FINISHED},
        {APP_STATE_INSPECTING, APP_EVENT_RC_NEUTRAL_CONFIRMED},
        {APP_STATE_EMERGENCY, APP_EVENT_RC_INSPECT_START},
    };
    size_t index;

    g_tests_run++;
    for (index = 0; index < (sizeof(cases) / sizeof(cases[0])); index++)
    {
        const app_state_machine_t machine = machine_in(cases[index].state);
        const app_transition_result_t result = transition(
            machine, cases[index].event);
        EXPECT_EQ(machine.state, result.machine.state);
        EXPECT_EQ(machine.selected_mode, result.machine.selected_mode);
        EXPECT_EQ(APP_ACTION_NONE, result.actions);
    }
}

static void test_invalid_values_produce_safe_fault(void)
{
    app_state_machine_t machine = machine_in((app_state_t)99);
    app_transition_result_t result;

    g_tests_run++;
    result = transition(machine, APP_EVENT_SELF_CHECK_PASSED);
    EXPECT_EQ(APP_STATE_FAULT, result.machine.state);
    EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);

    machine = machine_in(APP_STATE_REMOTE);
    result = transition(machine, (app_event_t)APP_EVENT_COUNT);
    EXPECT_EQ(APP_STATE_FAULT, result.machine.state);

    machine = machine_in(APP_STATE_REMOTE);
    machine.selected_mode = (app_mode_selection_t)99;
    result = transition(machine, APP_EVENT_RC_NEUTRAL_CONFIRMED);
    EXPECT_EQ(APP_STATE_FAULT, result.machine.state);
}

static void test_fault_cannot_be_cleared_by_ordinary_events(void)
{
    const app_event_t events[] = {
        APP_EVENT_SELF_CHECK_PASSED,
        APP_EVENT_RC_SELECT_REMOTE,
        APP_EVENT_RC_SELECT_STANDBY,
        APP_EVENT_RC_NEUTRAL_CONFIRMED,
        APP_EVENT_RC_EMERGENCY_RELEASED,
        APP_EVENT_JETSON_INSPECT_ACCEPTED,
        APP_EVENT_JETSON_INSPECT_FINISHED,
    };
    size_t index;

    g_tests_run++;
    for (index = 0; index < (sizeof(events) / sizeof(events[0])); index++)
    {
        const app_transition_result_t result = transition(
            machine_in(APP_STATE_FAULT), events[index]);
        EXPECT_EQ(APP_STATE_FAULT, result.machine.state);
        EXPECT_ACTION_SET(APP_ACTION_CLEAR_ALL_MOTION_COMMANDS, result.actions);
        expect_no_motion_permission(&result.machine);
    }
}

int main(void)
{
    app_state_machine_t boot = app_state_machine_initial();
    g_tests_run++;
    boot = transition(boot, APP_EVENT_RC_SELECT_STANDBY).machine;
    boot = transition(boot, APP_EVENT_RC_EMERGENCY_ASSERTED).machine;
    boot = transition(boot, APP_EVENT_RC_EMERGENCY_ASSERTED).machine;
    boot = transition(boot, APP_EVENT_RC_EMERGENCY_RELEASED).machine;
    EXPECT_EQ(APP_STATE_BOOT_SAFE, boot.state);
    EXPECT_EQ(APP_STATE_BOOT_SAFE,
              transition(boot, APP_EVENT_RC_INSPECT_START).machine.state);
    test_power_on_defaults_to_boot_safe();
    test_self_check_failure_enters_latched_fault();
    test_boot_remote_selection_requires_arming_and_neutral();
    test_boot_standby_selection_enters_standby();
    test_standby_remote_selection_requires_arming();
    test_remote_arming_can_return_to_standby();
    test_remote_arming_rejects_neutral_when_mode_is_not_remote();
    test_ch5_only_requests_inspection_from_standby();
    test_jetson_acceptance_is_required_for_inspecting();
    test_inspection_request_reject_timeout_and_cancel_return_to_standby();
    test_inspection_request_remote_selection_cancels_and_arms();
    test_inspecting_exits_safely_for_all_requested_events();
    test_emergency_assertion_preempts_every_nonfault_state();
    test_emergency_release_never_bypasses_remote_arming();
    test_rc_signal_loss_faults_each_required_state();
    test_motor_fault_enters_fault_from_motion_states();
    test_permissions_are_exclusive_and_derived_only_from_state();
    test_every_control_source_switch_clears_motion_commands();
    test_known_irrelevant_events_are_ignored_without_actions();
    test_invalid_values_produce_safe_fault();
    test_fault_cannot_be_cleared_by_ordinary_events();

    if (g_failures != 0)
    {
        (void)printf("%d test assertion(s) failed across %d tests\n",
                     g_failures, g_tests_run);
        return 1;
    }

    (void)printf("all %d app_state_machine tests passed\n", g_tests_run);
    return 0;
}
