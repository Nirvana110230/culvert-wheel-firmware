#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#define RC_INPUT_PULSE_MIN_US          500u
#define RC_INPUT_PULSE_MAX_US          2500u
#define RC_INPUT_AXIS_CENTER_US        1500u
#define RC_INPUT_AXIS_DEADBAND_US      50u
#define RC_INPUT_SWITCH_LOW_MIN_US     900u
#define RC_INPUT_SWITCH_LOW_MAX_US     1100u
#define RC_INPUT_SWITCH_MIDDLE_MIN_US  1400u
#define RC_INPUT_SWITCH_MIDDLE_MAX_US  1600u
#define RC_INPUT_SWITCH_HIGH_MIN_US    1900u
#define RC_INPUT_SWITCH_HIGH_MAX_US    2100u
#define RC_INPUT_STABLE_SAMPLES        3u
#define RC_INPUT_TIMEOUT_MS            100u

typedef enum
{
    RC_INPUT_CH1 = 0,
    RC_INPUT_CH2,
    RC_INPUT_CH5,
    RC_INPUT_CH6,
    RC_INPUT_CH7,
    RC_INPUT_CHANNEL_COUNT
} rc_input_channel_t;

typedef enum
{
    RC_INPUT_MODE_INVALID = 0,
    RC_INPUT_MODE_REMOTE,
    RC_INPUT_MODE_STANDBY
} rc_input_mode_t;

typedef enum
{
    RC_INPUT_EVENT_NONE = 0,
    RC_INPUT_EVENT_INSPECT_START = (1u << 0),
    RC_INPUT_EVENT_SELECT_REMOTE = (1u << 1),
    RC_INPUT_EVENT_SELECT_STANDBY = (1u << 2),
    RC_INPUT_EVENT_EMERGENCY_ASSERTED = (1u << 3),
    RC_INPUT_EVENT_EMERGENCY_RELEASED = (1u << 4),
    RC_INPUT_EVENT_NEUTRAL_CONFIRMED = (1u << 5),
    RC_INPUT_EVENT_SIGNAL_LOST = (1u << 6)
} rc_input_event_t;

typedef struct
{
    int16_t ch1_normalized;
    int16_t ch2_normalized;
    rc_input_mode_t mode;
    bool emergency_asserted;
    bool signal_valid;
    bool channel_valid[RC_INPUT_CHANNEL_COUNT];
    uint16_t pulse_us[RC_INPUT_CHANNEL_COUNT];
    uint32_t last_valid_ms[RC_INPUT_CHANNEL_COUNT];
} rc_input_snapshot_t;

typedef struct
{
    rc_input_snapshot_t snapshot;
    bool channel_seen[RC_INPUT_CHANNEL_COUNT];
    uint8_t ch5_candidate;
    uint8_t ch5_count;
    bool ch5_armed;
    uint8_t ch6_candidate;
    uint8_t ch6_count;
    uint8_t ch7_release_count;
    uint8_t neutral_count;
    uint8_t neutral_pair_mask;
    bool neutral_reported;
    bool signal_lost_reported;
    bool monitoring_started;
    bool ignore_inspection_channel;
    uint32_t monitoring_start_ms;
} rc_input_t;

void rc_input_init(rc_input_t *input);
void rc_input_restart_neutral_confirmation(rc_input_t *input);

uint32_t rc_input_process_sample(rc_input_t *input,
                                 rc_input_channel_t channel,
                                 uint16_t pulse_us,
                                 uint32_t timestamp_ms);

uint32_t rc_input_poll(rc_input_t *input, uint32_t now_ms);

rc_input_snapshot_t rc_input_get_snapshot(const rc_input_t *input);

#endif
