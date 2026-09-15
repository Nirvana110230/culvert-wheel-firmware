#include "rc_capture.h"
#include "rc_input.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned int tests_run;

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__,   \
                    #condition);                                                \
            exit(EXIT_FAILURE);                                                 \
        }                                                                       \
    } while (false)

#define RUN(test_function)                                                      \
    do                                                                          \
    {                                                                           \
        test_function();                                                        \
        ++tests_run;                                                            \
    } while (false)

static void submit_nominal_set(rc_input_t *input, uint32_t now_ms)
{
    (void)rc_input_process_sample(input, RC_INPUT_CH1, 1500u, now_ms);
    (void)rc_input_process_sample(input, RC_INPUT_CH2, 1500u, now_ms);
    (void)rc_input_process_sample(input, RC_INPUT_CH5, 1500u, now_ms);
    (void)rc_input_process_sample(input, RC_INPUT_CH6, 1000u, now_ms);
    (void)rc_input_process_sample(input, RC_INPUT_CH7, 1000u, now_ms);
}

static void test_normal_pulse_width(void)
{
    CHECK(rc_capture_elapsed_ticks(1000u, 2500u) == 1500u);
}

static void test_wrapped_pulse_width(void)
{
    CHECK(rc_capture_elapsed_ticks(65000u, 964u) == 1500u);
}

static void test_pulse_below_minimum(void)
{
    rc_input_t input;
    rc_input_init(&input);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH1, 499u, 1u) ==
          RC_INPUT_EVENT_NONE);
    CHECK(!rc_input_get_snapshot(&input).channel_valid[RC_INPUT_CH1]);
}

static void test_pulse_above_maximum(void)
{
    rc_input_t input;
    rc_input_init(&input);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH2, 2501u, 1u) ==
          RC_INPUT_EVENT_NONE);
    CHECK(!rc_input_get_snapshot(&input).channel_valid[RC_INPUT_CH2]);
}

static void test_axis_center_and_deadband(void)
{
    rc_input_t input;
    rc_input_init(&input);
    (void)rc_input_process_sample(&input, RC_INPUT_CH1, 1450u, 1u);
    (void)rc_input_process_sample(&input, RC_INPUT_CH2, 1550u, 1u);
    CHECK(rc_input_get_snapshot(&input).ch1_normalized == 0);
    CHECK(rc_input_get_snapshot(&input).ch2_normalized == 0);
}

static void test_axis_direction_and_saturation(void)
{
    rc_input_t input;
    rc_input_init(&input);
    (void)rc_input_process_sample(&input, RC_INPUT_CH1, 1000u, 1u);
    (void)rc_input_process_sample(&input, RC_INPUT_CH2, 1000u, 1u);
    CHECK(rc_input_get_snapshot(&input).ch1_normalized == 1000);
    CHECK(rc_input_get_snapshot(&input).ch2_normalized == -1000);
    (void)rc_input_process_sample(&input, RC_INPUT_CH1, 2100u, 2u);
    (void)rc_input_process_sample(&input, RC_INPUT_CH2, 2100u, 2u);
    CHECK(rc_input_get_snapshot(&input).ch1_normalized == -1000);
    CHECK(rc_input_get_snapshot(&input).ch2_normalized == 1000);
}

static void test_ch5_first_stable_trigger(void)
{
    rc_input_t input;
    uint32_t events = 0u;
    rc_input_init(&input);
    events |= rc_input_process_sample(&input, RC_INPUT_CH5, 2000u, 1u);
    events |= rc_input_process_sample(&input, RC_INPUT_CH5, 2000u, 2u);
    CHECK((events & RC_INPUT_EVENT_INSPECT_START) == 0u);
    events |= rc_input_process_sample(&input, RC_INPUT_CH5, 2000u, 3u);
    CHECK((events & RC_INPUT_EVENT_INSPECT_START) != 0u);
}

static void test_ch5_hold_does_not_repeat(void)
{
    rc_input_t input;
    rc_input_init(&input);
    for (uint32_t i = 0u; i < 3u; ++i)
    {
        (void)rc_input_process_sample(&input, RC_INPUT_CH5, 2000u, i);
    }
    for (uint32_t i = 3u; i < 20u; ++i)
    {
        CHECK((rc_input_process_sample(&input, RC_INPUT_CH5, 2000u, i) &
               RC_INPUT_EVENT_INSPECT_START) == 0u);
    }
}

static void test_ch5_release_and_retrigger(void)
{
    rc_input_t input;
    rc_input_init(&input);
    for (uint32_t i = 0u; i < 3u; ++i)
    {
        (void)rc_input_process_sample(&input, RC_INPUT_CH5, 2000u, i);
    }
    for (uint32_t i = 3u; i < 6u; ++i)
    {
        (void)rc_input_process_sample(&input, RC_INPUT_CH5, 1500u, i);
    }
    CHECK((rc_input_process_sample(&input, RC_INPUT_CH5, 2000u, 6u) &
           RC_INPUT_EVENT_INSPECT_START) == 0u);
    CHECK((rc_input_process_sample(&input, RC_INPUT_CH5, 2000u, 7u) &
           RC_INPUT_EVENT_INSPECT_START) == 0u);
    CHECK((rc_input_process_sample(&input, RC_INPUT_CH5, 2000u, 8u) &
           RC_INPUT_EVENT_INSPECT_START) != 0u);
}

static void test_ch6_remote_stable(void)
{
    rc_input_t input;
    rc_input_init(&input);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH6, 1000u, 1u) == 0u);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH6, 1000u, 2u) == 0u);
    CHECK((rc_input_process_sample(&input, RC_INPUT_CH6, 1000u, 3u) &
           RC_INPUT_EVENT_SELECT_REMOTE) != 0u);
    CHECK(rc_input_get_snapshot(&input).mode == RC_INPUT_MODE_REMOTE);
}

static void test_ch6_standby_stable(void)
{
    rc_input_t input;
    rc_input_init(&input);
    for (uint32_t i = 0u; i < 3u; ++i)
    {
        CHECK((rc_input_process_sample(&input, RC_INPUT_CH6, 2000u, i) &
               RC_INPUT_EVENT_SELECT_STANDBY) == (i == 2u ?
                   RC_INPUT_EVENT_SELECT_STANDBY : 0u));
    }
    CHECK(rc_input_get_snapshot(&input).mode == RC_INPUT_MODE_STANDBY);
}

static void test_ch6_middle_and_invalid_do_not_switch(void)
{
    rc_input_t input;
    rc_input_init(&input);
    for (uint32_t i = 0u; i < 3u; ++i)
    {
        (void)rc_input_process_sample(&input, RC_INPUT_CH6, 1000u, i);
    }
    for (uint32_t i = 3u; i < 9u; ++i)
    {
        const uint16_t pulse = (i < 6u) ? 1500u : 2200u;
        CHECK(rc_input_process_sample(&input, RC_INPUT_CH6, pulse, i) == 0u);
    }
    CHECK(rc_input_get_snapshot(&input).mode == RC_INPUT_MODE_REMOTE);
}

static void test_ch6_invalid_sample_breaks_stability(void)
{
    rc_input_t input;
    rc_input_init(&input);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH6, 1000u, 0u) == 0u);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH6, 1000u, 1u) == 0u);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH6, 1500u, 2u) == 0u);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH6, 1000u, 3u) == 0u);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH6, 1000u, 4u) == 0u);
    CHECK((rc_input_process_sample(&input, RC_INPUT_CH6, 1000u, 5u) &
           RC_INPUT_EVENT_SELECT_REMOTE) != 0u);
}

static void test_ch7_single_sample_asserts(void)
{
    rc_input_t input;
    rc_input_init(&input);
    CHECK((rc_input_process_sample(&input, RC_INPUT_CH7, 2000u, 1u) &
           RC_INPUT_EVENT_EMERGENCY_ASSERTED) != 0u);
    CHECK(rc_input_get_snapshot(&input).emergency_asserted);
}

static void test_ch7_release_requires_stability(void)
{
    rc_input_t input;
    rc_input_init(&input);
    (void)rc_input_process_sample(&input, RC_INPUT_CH7, 2000u, 0u);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH7, 1000u, 1u) == 0u);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH7, 1000u, 2u) == 0u);
    CHECK((rc_input_process_sample(&input, RC_INPUT_CH7, 1000u, 3u) &
           RC_INPUT_EVENT_EMERGENCY_RELEASED) != 0u);
    CHECK(!rc_input_get_snapshot(&input).emergency_asserted);
}

static void test_ch7_invalid_sample_breaks_release_stability(void)
{
    rc_input_t input;
    rc_input_init(&input);
    (void)rc_input_process_sample(&input, RC_INPUT_CH7, 2000u, 0u);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH7, 1000u, 1u) == 0u);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH7, 1000u, 2u) == 0u);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH7, 1500u, 3u) == 0u);
    CHECK(rc_input_process_sample(&input, RC_INPUT_CH7, 1000u, 4u) == 0u);
    CHECK(rc_input_get_snapshot(&input).emergency_asserted);
}

static void test_ch7_loss_cannot_release(void)
{
    rc_input_t input;
    rc_input_init(&input);
    (void)rc_input_process_sample(&input, RC_INPUT_CH7, 2000u, 0u);
    submit_nominal_set(&input, 1u);
    CHECK((rc_input_poll(&input, RC_INPUT_TIMEOUT_MS + 2u) &
           RC_INPUT_EVENT_SIGNAL_LOST) != 0u);
    CHECK(rc_input_get_snapshot(&input).emergency_asserted);
}

static void test_single_channel_timeout(void)
{
    rc_input_t input;
    rc_input_init(&input);
    submit_nominal_set(&input, 0u);
    (void)rc_input_process_sample(&input, RC_INPUT_CH2, 1500u, 80u);
    (void)rc_input_process_sample(&input, RC_INPUT_CH5, 1500u, 80u);
    (void)rc_input_process_sample(&input, RC_INPUT_CH6, 1000u, 80u);
    (void)rc_input_process_sample(&input, RC_INPUT_CH7, 1000u, 80u);
    CHECK((rc_input_poll(&input, 101u) & RC_INPUT_EVENT_SIGNAL_LOST) != 0u);
    CHECK(!rc_input_get_snapshot(&input).channel_valid[RC_INPUT_CH1]);
}

static void test_startup_signal_loss_waits_for_timeout(void)
{
    rc_input_t input;
    rc_input_init(&input);
    CHECK(rc_input_poll(&input, 10u) == RC_INPUT_EVENT_NONE);
    CHECK(rc_input_poll(&input, 10u + RC_INPUT_TIMEOUT_MS) ==
          RC_INPUT_EVENT_NONE);
    CHECK((rc_input_poll(&input, 11u + RC_INPUT_TIMEOUT_MS) &
           RC_INPUT_EVENT_SIGNAL_LOST) != 0u);
}

static void test_multiple_channel_timeout_is_single_edge(void)
{
    rc_input_t input;
    rc_input_init(&input);
    submit_nominal_set(&input, 0u);
    CHECK((rc_input_poll(&input, 101u) & RC_INPUT_EVENT_SIGNAL_LOST) != 0u);
    CHECK(rc_input_poll(&input, 102u) == RC_INPUT_EVENT_NONE);
}

static void test_signal_recovery_rebuilds_validity(void)
{
    rc_input_t input;
    rc_input_init(&input);
    submit_nominal_set(&input, 0u);
    (void)rc_input_poll(&input, 101u);
    CHECK(!rc_input_get_snapshot(&input).signal_valid);
    submit_nominal_set(&input, 102u);
    CHECK(rc_input_get_snapshot(&input).signal_valid);
    CHECK((rc_input_poll(&input, 102u) & RC_INPUT_EVENT_SIGNAL_LOST) == 0u);
}

static void test_timestamp_wraparound(void)
{
    rc_input_t input;
    rc_input_init(&input);
    submit_nominal_set(&input, UINT32_MAX - 50u);
    CHECK(rc_input_poll(&input, 25u) == RC_INPUT_EVENT_NONE);
    CHECK((rc_input_poll(&input, 51u) & RC_INPUT_EVENT_SIGNAL_LOST) != 0u);
}

static void test_invalid_channel_number(void)
{
    rc_input_t input;
    rc_input_init(&input);
    CHECK(rc_input_process_sample(&input, (rc_input_channel_t)99, 1500u, 0u) ==
          RC_INPUT_EVENT_NONE);
}

static void test_boundary_values(void)
{
    rc_input_t input;
    rc_input_init(&input);
    (void)rc_input_process_sample(&input, RC_INPUT_CH1, 500u, 0u);
    CHECK(rc_input_get_snapshot(&input).channel_valid[RC_INPUT_CH1]);
    (void)rc_input_process_sample(&input, RC_INPUT_CH2, 2500u, 1u);
    CHECK(rc_input_get_snapshot(&input).channel_valid[RC_INPUT_CH2]);
    CHECK(rc_input_get_snapshot(&input).ch1_normalized == 1000);
    CHECK(rc_input_get_snapshot(&input).ch2_normalized == 1000);
}

static void test_lost_signal_does_not_hold_nonzero_axes(void)
{
    rc_input_t input;
    rc_input_init(&input);
    submit_nominal_set(&input, 0u);
    (void)rc_input_process_sample(&input, RC_INPUT_CH1, 1000u, 1u);
    (void)rc_input_process_sample(&input, RC_INPUT_CH2, 2000u, 1u);
    CHECK(rc_input_get_snapshot(&input).ch1_normalized != 0);
    CHECK(rc_input_get_snapshot(&input).ch2_normalized != 0);
    (void)rc_input_poll(&input, 102u);
    CHECK(rc_input_get_snapshot(&input).ch1_normalized == 0);
    CHECK(rc_input_get_snapshot(&input).ch2_normalized == 0);
}

static void test_neutral_confirmation_requires_complete_stable_pairs(void)
{
    rc_input_t input;
    uint32_t events = 0u;
    rc_input_init(&input);
    for (uint32_t i = 0u; i < 2u; ++i)
    {
        events |= rc_input_process_sample(&input, RC_INPUT_CH1, 1500u, i);
        events |= rc_input_process_sample(&input, RC_INPUT_CH2, 1500u, i);
    }
    CHECK((events & RC_INPUT_EVENT_NEUTRAL_CONFIRMED) == 0u);
    events |= rc_input_process_sample(&input, RC_INPUT_CH1, 1500u, 2u);
    CHECK((events & RC_INPUT_EVENT_NEUTRAL_CONFIRMED) == 0u);
    events |= rc_input_process_sample(&input, RC_INPUT_CH2, 1500u, 2u);
    CHECK((events & RC_INPUT_EVENT_NEUTRAL_CONFIRMED) != 0u);
}

static void test_invalid_axis_immediately_clears_command(void)
{
    rc_input_t input;
    rc_input_init(&input);
    (void)rc_input_process_sample(&input, RC_INPUT_CH1, 1000u, 0u);
    CHECK(rc_input_get_snapshot(&input).ch1_normalized == 1000);
    (void)rc_input_process_sample(&input, RC_INPUT_CH1, 3000u, 1u);
    CHECK(rc_input_get_snapshot(&input).ch1_normalized == 0);
}

int main(void)
{
    RUN(test_normal_pulse_width);
    RUN(test_wrapped_pulse_width);
    RUN(test_pulse_below_minimum);
    RUN(test_pulse_above_maximum);
    RUN(test_axis_center_and_deadband);
    RUN(test_axis_direction_and_saturation);
    RUN(test_ch5_first_stable_trigger);
    RUN(test_ch5_hold_does_not_repeat);
    RUN(test_ch5_release_and_retrigger);
    RUN(test_ch6_remote_stable);
    RUN(test_ch6_standby_stable);
    RUN(test_ch6_middle_and_invalid_do_not_switch);
    RUN(test_ch6_invalid_sample_breaks_stability);
    RUN(test_ch7_single_sample_asserts);
    RUN(test_ch7_release_requires_stability);
    RUN(test_ch7_invalid_sample_breaks_release_stability);
    RUN(test_ch7_loss_cannot_release);
    RUN(test_single_channel_timeout);
    RUN(test_startup_signal_loss_waits_for_timeout);
    RUN(test_multiple_channel_timeout_is_single_edge);
    RUN(test_signal_recovery_rebuilds_validity);
    RUN(test_timestamp_wraparound);
    RUN(test_invalid_channel_number);
    RUN(test_boundary_values);
    RUN(test_lost_signal_does_not_hold_nonzero_axes);
    RUN(test_neutral_confirmation_requires_complete_stable_pairs);
    RUN(test_invalid_axis_immediately_clears_command);

    printf("all %u rc_input tests passed\n", tests_run);
    return EXIT_SUCCESS;
}
