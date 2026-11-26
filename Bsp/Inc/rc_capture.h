#ifndef RC_CAPTURE_H
#define RC_CAPTURE_H

#include "rc_input.h"

#include <stdbool.h>
#include <stdint.h>

#define RC_CAPTURE_QUEUE_CAPACITY 16u

typedef struct
{
    uint16_t pulse_us;
    uint32_t timestamp_ms;
    uint32_t sequence;
} rc_capture_channel_snapshot_t;

typedef struct
{
    rc_capture_channel_snapshot_t channels[RC_INPUT_CHANNEL_COUNT];
    uint32_t dropped_samples;
} rc_capture_snapshot_t;

typedef struct
{
    rc_input_channel_t channel;
    uint16_t pulse_us;
    uint32_t timestamp_ms;
} rc_capture_sample_t;

static inline uint16_t rc_capture_elapsed_ticks(uint16_t rising,
                                                uint16_t falling)
{
    return (uint16_t)(falling - rising);
}

bool rc_capture_start(bool queue_inspection_channel);
void rc_capture_stop(void);
void rc_capture_read_snapshot(rc_capture_snapshot_t *snapshot);
bool rc_capture_take_sample(rc_capture_sample_t *sample);

#endif
