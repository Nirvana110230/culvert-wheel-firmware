#include "rc_input.h"

#include <stddef.h>
#include <string.h>

typedef enum
{
    SWITCH_POSITION_INVALID = 0,
    SWITCH_POSITION_LOW,
    SWITCH_POSITION_MIDDLE,
    SWITCH_POSITION_HIGH
} switch_position_t;

static bool pulse_in_range(uint16_t pulse_us, uint16_t minimum, uint16_t maximum)
{
    return (pulse_us >= minimum) && (pulse_us <= maximum);
}

static switch_position_t classify_switch(uint16_t pulse_us)
{
    if (pulse_in_range(pulse_us, RC_INPUT_SWITCH_LOW_MIN_US,
                       RC_INPUT_SWITCH_LOW_MAX_US))
    {
        return SWITCH_POSITION_LOW;
    }
    if (pulse_in_range(pulse_us, RC_INPUT_SWITCH_MIDDLE_MIN_US,
                       RC_INPUT_SWITCH_MIDDLE_MAX_US))
    {
        return SWITCH_POSITION_MIDDLE;
    }
    if (pulse_in_range(pulse_us, RC_INPUT_SWITCH_HIGH_MIN_US,
                       RC_INPUT_SWITCH_HIGH_MAX_US))
    {
        return SWITCH_POSITION_HIGH;
    }
    return SWITCH_POSITION_INVALID;
}

static bool channel_sample_is_valid(rc_input_channel_t channel,
                                    uint16_t pulse_us,
                                    switch_position_t position)
{
    if (!pulse_in_range(pulse_us, RC_INPUT_PULSE_MIN_US,
                        RC_INPUT_PULSE_MAX_US))
    {
        return false;
    }

    if ((channel == RC_INPUT_CH1) || (channel == RC_INPUT_CH2))
    {
        return true;
    }
    if (channel == RC_INPUT_CH5)
    {
        return position != SWITCH_POSITION_INVALID;
    }
    if ((channel == RC_INPUT_CH6) || (channel == RC_INPUT_CH7))
    {
        return (position == SWITCH_POSITION_LOW) ||
               (position == SWITCH_POSITION_HIGH);
    }
    return false;
}

static int16_t normalize_axis(uint16_t pulse_us, bool reverse)
{
    int32_t delta = (int32_t)pulse_us - (int32_t)RC_INPUT_AXIS_CENTER_US;
    int32_t normalized;

    if ((delta >= -(int32_t)RC_INPUT_AXIS_DEADBAND_US) &&
        (delta <= (int32_t)RC_INPUT_AXIS_DEADBAND_US))
    {
        return 0;
    }

    normalized = delta * 2;
    if (normalized > 1000)
    {
        normalized = 1000;
    }
    else if (normalized < -1000)
    {
        normalized = -1000;
    }

    if (reverse)
    {
        normalized = -normalized;
    }
    return (int16_t)normalized;
}

static bool all_channels_fresh(const rc_input_t *input, uint32_t now_ms)
{
    uint32_t channel;

    for (channel = 0u; channel < (uint32_t)RC_INPUT_CHANNEL_COUNT; ++channel)
    {
        if (!input->channel_seen[channel] ||
            !input->snapshot.channel_valid[channel] ||
            ((uint32_t)(now_ms - input->snapshot.last_valid_ms[channel]) >
             RC_INPUT_TIMEOUT_MS))
        {
            return false;
        }
    }
    return true;
}

static uint32_t update_neutral_confirmation(rc_input_t *input,
                                            rc_input_channel_t channel)
{
    const uint8_t bit = (channel == RC_INPUT_CH1) ? 1u : 2u;
    const bool both_neutral = (input->snapshot.ch1_normalized == 0) &&
                              (input->snapshot.ch2_normalized == 0);

    if (!both_neutral)
    {
        input->neutral_count = 0u;
        input->neutral_pair_mask = 0u;
        input->neutral_reported = false;
        return RC_INPUT_EVENT_NONE;
    }

    input->neutral_pair_mask = (uint8_t)(input->neutral_pair_mask | bit);
    if (input->neutral_pair_mask != 3u)
    {
        return RC_INPUT_EVENT_NONE;
    }

    input->neutral_pair_mask = 0u;
    if (input->neutral_count < RC_INPUT_STABLE_SAMPLES)
    {
        ++input->neutral_count;
    }
    if ((input->neutral_count == RC_INPUT_STABLE_SAMPLES) &&
        !input->neutral_reported)
    {
        input->neutral_reported = true;
        return RC_INPUT_EVENT_NEUTRAL_CONFIRMED;
    }
    return RC_INPUT_EVENT_NONE;
}

static uint32_t process_ch5(rc_input_t *input, switch_position_t position)
{
    if (position != (switch_position_t)input->ch5_candidate)
    {
        input->ch5_candidate = (uint8_t)position;
        input->ch5_count = 1u;
    }
    else if (input->ch5_count < RC_INPUT_STABLE_SAMPLES)
    {
        ++input->ch5_count;
    }

    if (input->ch5_count < RC_INPUT_STABLE_SAMPLES)
    {
        return RC_INPUT_EVENT_NONE;
    }
    if ((position == SWITCH_POSITION_HIGH) && input->ch5_armed)
    {
        input->ch5_armed = false;
        return RC_INPUT_EVENT_INSPECT_START;
    }
    if (position != SWITCH_POSITION_HIGH)
    {
        input->ch5_armed = true;
    }
    return RC_INPUT_EVENT_NONE;
}

static uint32_t process_ch6(rc_input_t *input, switch_position_t position)
{
    rc_input_mode_t mode;

    if (position != (switch_position_t)input->ch6_candidate)
    {
        input->ch6_candidate = (uint8_t)position;
        input->ch6_count = 1u;
    }
    else if (input->ch6_count < RC_INPUT_STABLE_SAMPLES)
    {
        ++input->ch6_count;
    }

    if (input->ch6_count < RC_INPUT_STABLE_SAMPLES)
    {
        return RC_INPUT_EVENT_NONE;
    }

    mode = (position == SWITCH_POSITION_LOW) ? RC_INPUT_MODE_REMOTE :
                                               RC_INPUT_MODE_STANDBY;
    if (input->snapshot.mode == mode)
    {
        return RC_INPUT_EVENT_NONE;
    }
    input->snapshot.mode = mode;
    return (mode == RC_INPUT_MODE_REMOTE) ? RC_INPUT_EVENT_SELECT_REMOTE :
                                            RC_INPUT_EVENT_SELECT_STANDBY;
}

static uint32_t process_ch7(rc_input_t *input, switch_position_t position)
{
    if (position == SWITCH_POSITION_HIGH)
    {
        input->ch7_release_count = 0u;
        if (!input->snapshot.emergency_asserted)
        {
            input->snapshot.emergency_asserted = true;
            return RC_INPUT_EVENT_EMERGENCY_ASSERTED;
        }
        return RC_INPUT_EVENT_NONE;
    }

    if (!input->snapshot.emergency_asserted)
    {
        input->ch7_release_count = 0u;
        return RC_INPUT_EVENT_NONE;
    }
    if (input->ch7_release_count < RC_INPUT_STABLE_SAMPLES)
    {
        ++input->ch7_release_count;
    }
    if (input->ch7_release_count == RC_INPUT_STABLE_SAMPLES)
    {
        input->ch7_release_count = 0u;
        input->snapshot.emergency_asserted = false;
        return RC_INPUT_EVENT_EMERGENCY_RELEASED;
    }
    return RC_INPUT_EVENT_NONE;
}

void rc_input_init(rc_input_t *input)
{
    if (input == NULL)
    {
        return;
    }
    (void)memset(input, 0, sizeof(*input));
    input->ch5_armed = true;
    input->snapshot.mode = RC_INPUT_MODE_INVALID;
}

static void invalidate_channel_sample(rc_input_t *input,
                                      rc_input_channel_t channel)
{
    input->snapshot.channel_valid[channel] = false;
    input->snapshot.signal_valid = false;

    switch (channel)
    {
        case RC_INPUT_CH1:
            input->snapshot.ch1_normalized = 0;
            input->neutral_count = 0u;
            input->neutral_pair_mask = 0u;
            input->neutral_reported = false;
            break;
        case RC_INPUT_CH2:
            input->snapshot.ch2_normalized = 0;
            input->neutral_count = 0u;
            input->neutral_pair_mask = 0u;
            input->neutral_reported = false;
            break;
        case RC_INPUT_CH5:
            input->ch5_candidate = (uint8_t)SWITCH_POSITION_INVALID;
            input->ch5_count = 0u;
            break;
        case RC_INPUT_CH6:
            input->ch6_candidate = (uint8_t)SWITCH_POSITION_INVALID;
            input->ch6_count = 0u;
            break;
        case RC_INPUT_CH7:
            input->ch7_release_count = 0u;
            break;
        case RC_INPUT_CHANNEL_COUNT:
        default:
            break;
    }
}

uint32_t rc_input_process_sample(rc_input_t *input,
                                 rc_input_channel_t channel,
                                 uint16_t pulse_us,
                                 uint32_t timestamp_ms)
{
    switch_position_t position;
    uint32_t events = RC_INPUT_EVENT_NONE;

    if ((input == NULL) || ((uint32_t)channel >= RC_INPUT_CHANNEL_COUNT))
    {
        return RC_INPUT_EVENT_NONE;
    }

    position = classify_switch(pulse_us);
    if (!channel_sample_is_valid(channel, pulse_us, position))
    {
        invalidate_channel_sample(input, channel);
        return RC_INPUT_EVENT_NONE;
    }

    if (!input->monitoring_started)
    {
        input->monitoring_started = true;
        input->monitoring_start_ms = timestamp_ms;
    }

    input->channel_seen[channel] = true;
    input->snapshot.channel_valid[channel] = true;
    input->snapshot.pulse_us[channel] = pulse_us;
    input->snapshot.last_valid_ms[channel] = timestamp_ms;

    switch (channel)
    {
        case RC_INPUT_CH1:
            input->snapshot.ch1_normalized = normalize_axis(pulse_us, true);
            events |= update_neutral_confirmation(input, channel);
            break;
        case RC_INPUT_CH2:
            input->snapshot.ch2_normalized = normalize_axis(pulse_us, false);
            events |= update_neutral_confirmation(input, channel);
            break;
        case RC_INPUT_CH5:
            events |= process_ch5(input, position);
            break;
        case RC_INPUT_CH6:
            events |= process_ch6(input, position);
            break;
        case RC_INPUT_CH7:
            events |= process_ch7(input, position);
            break;
        case RC_INPUT_CHANNEL_COUNT:
        default:
            break;
    }

    input->snapshot.signal_valid = all_channels_fresh(input, timestamp_ms);
    if (input->snapshot.signal_valid)
    {
        input->signal_lost_reported = false;
    }
    return events;
}

uint32_t rc_input_poll(rc_input_t *input, uint32_t now_ms)
{
    uint32_t channel;
    bool fresh;
    bool timeout_detected;

    if (input == NULL)
    {
        return RC_INPUT_EVENT_NONE;
    }

    if (!input->monitoring_started)
    {
        input->monitoring_started = true;
        input->monitoring_start_ms = now_ms;
        return RC_INPUT_EVENT_NONE;
    }

    fresh = true;
    timeout_detected = false;
    for (channel = 0u; channel < (uint32_t)RC_INPUT_CHANNEL_COUNT; ++channel)
    {
        const bool channel_fresh = input->channel_seen[channel] &&
            ((uint32_t)(now_ms - input->snapshot.last_valid_ms[channel]) <=
             RC_INPUT_TIMEOUT_MS);
        const bool channel_timed_out = input->channel_seen[channel] ?
            !channel_fresh :
            ((uint32_t)(now_ms - input->monitoring_start_ms) >
             RC_INPUT_TIMEOUT_MS);
        input->snapshot.channel_valid[channel] = channel_fresh;
        fresh = fresh && channel_fresh;
        timeout_detected = timeout_detected || channel_timed_out;
    }
    input->snapshot.signal_valid = fresh;
    if (fresh)
    {
        input->signal_lost_reported = false;
        return RC_INPUT_EVENT_NONE;
    }
    if (!timeout_detected)
    {
        return RC_INPUT_EVENT_NONE;
    }

    input->snapshot.ch1_normalized = 0;
    input->snapshot.ch2_normalized = 0;
    input->neutral_count = 0u;
    input->neutral_pair_mask = 0u;
    input->neutral_reported = false;
    input->ch7_release_count = 0u;
    if (!input->signal_lost_reported)
    {
        input->signal_lost_reported = true;
        return RC_INPUT_EVENT_SIGNAL_LOST;
    }
    return RC_INPUT_EVENT_NONE;
}

rc_input_snapshot_t rc_input_get_snapshot(const rc_input_t *input)
{
    rc_input_snapshot_t snapshot;

    (void)memset(&snapshot, 0, sizeof(snapshot));
    if (input != NULL)
    {
        snapshot = input->snapshot;
    }
    return snapshot;
}
