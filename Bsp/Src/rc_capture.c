#include "rc_capture.h"

#include "tim.h"

#include <stddef.h>

typedef struct
{
    volatile uint16_t pulse_us;
    volatile uint32_t timestamp_ms;
    volatile uint32_t sequence;
} rc_capture_shared_sample_t;

typedef struct
{
    uint16_t rising_count;
    uint32_t rising_ms;
    bool waiting_for_falling;
} rc_capture_edge_state_t;

static rc_capture_shared_sample_t shared_samples[RC_INPUT_CHANNEL_COUNT];
static rc_capture_edge_state_t edge_states[RC_INPUT_CHANNEL_COUNT];

static rc_capture_sample_t sample_queue[RC_CAPTURE_QUEUE_CAPACITY];
static volatile uint8_t queue_head;
static volatile uint8_t queue_tail;
static volatile uint32_t dropped_samples;
static bool inspection_channel_queued;

static void set_capture_polarity(TIM_HandleTypeDef *timer,
                                 uint32_t timer_channel,
                                 uint32_t polarity)
{
    __HAL_TIM_SET_CAPTUREPOLARITY(timer, timer_channel, polarity);
}

static void reset_capture_state(void)
{
    uint32_t channel;

    for (channel = 0u; channel < (uint32_t)RC_INPUT_CHANNEL_COUNT; ++channel)
    {
        shared_samples[channel].pulse_us = 0u;
        shared_samples[channel].timestamp_ms = 0u;
        shared_samples[channel].sequence = 0u;
        edge_states[channel].rising_count = 0u;
        edge_states[channel].waiting_for_falling = false;
    }
    queue_head = 0u;
    queue_tail = 0u;
    dropped_samples = 0u;

    set_capture_polarity(&htim8, TIM_CHANNEL_1,
                         TIM_INPUTCHANNELPOLARITY_RISING);
    set_capture_polarity(&htim8, TIM_CHANNEL_2,
                         TIM_INPUTCHANNELPOLARITY_RISING);
    set_capture_polarity(&htim8, TIM_CHANNEL_3,
                         TIM_INPUTCHANNELPOLARITY_RISING);
    set_capture_polarity(&htim8, TIM_CHANNEL_4,
                         TIM_INPUTCHANNELPOLARITY_RISING);
    set_capture_polarity(&htim3, TIM_CHANNEL_1,
                         TIM_INPUTCHANNELPOLARITY_RISING);
}

static bool start_channel(TIM_HandleTypeDef *timer, uint32_t timer_channel)
{
    return HAL_TIM_IC_Start_IT(timer, timer_channel) == HAL_OK;
}

bool rc_capture_start(bool queue_inspection_channel)
{
    inspection_channel_queued = queue_inspection_channel;
    reset_capture_state();

    if (!start_channel(&htim8, TIM_CHANNEL_1) ||
        !start_channel(&htim8, TIM_CHANNEL_2) ||
        !start_channel(&htim8, TIM_CHANNEL_3) ||
        !start_channel(&htim8, TIM_CHANNEL_4) ||
        !start_channel(&htim3, TIM_CHANNEL_1))
    {
        rc_capture_stop();
        return false;
    }
    return true;
}

void rc_capture_stop(void)
{
    (void)HAL_TIM_IC_Stop_IT(&htim8, TIM_CHANNEL_1);
    (void)HAL_TIM_IC_Stop_IT(&htim8, TIM_CHANNEL_2);
    (void)HAL_TIM_IC_Stop_IT(&htim8, TIM_CHANNEL_3);
    (void)HAL_TIM_IC_Stop_IT(&htim8, TIM_CHANNEL_4);
    (void)HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
}

void rc_capture_read_snapshot(rc_capture_snapshot_t *snapshot)
{
    uint32_t interrupt_state;
    uint32_t channel;

    if (snapshot == NULL)
    {
        return;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    __DMB();
    for (channel = 0u; channel < (uint32_t)RC_INPUT_CHANNEL_COUNT; ++channel)
    {
        snapshot->channels[channel].pulse_us = shared_samples[channel].pulse_us;
        snapshot->channels[channel].timestamp_ms =
            shared_samples[channel].timestamp_ms;
        snapshot->channels[channel].sequence = shared_samples[channel].sequence;
    }
    snapshot->dropped_samples = dropped_samples;
    __DMB();
    if (interrupt_state == 0u)
    {
        __enable_irq();
    }
}

bool rc_capture_take_sample(rc_capture_sample_t *sample)
{
    uint32_t interrupt_state;
    uint8_t tail;

    if (sample == NULL)
    {
        return false;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    __DMB();
    tail = queue_tail;
    if (tail == queue_head)
    {
        if (interrupt_state == 0u)
        {
            __enable_irq();
        }
        return false;
    }
    *sample = sample_queue[tail];
    queue_tail = (uint8_t)((tail + 1u) % RC_CAPTURE_QUEUE_CAPACITY);
    __DMB();
    if (interrupt_state == 0u)
    {
        __enable_irq();
    }
    return true;
}

static bool map_active_channel(TIM_HandleTypeDef *timer,
                               rc_input_channel_t *input_channel,
                               uint32_t *timer_channel)
{
    if (timer->Instance == TIM8)
    {
        switch (timer->Channel)
        {
            case HAL_TIM_ACTIVE_CHANNEL_1:
                *input_channel = RC_INPUT_CH1;
                *timer_channel = TIM_CHANNEL_1;
                return true;
            case HAL_TIM_ACTIVE_CHANNEL_2:
                *input_channel = RC_INPUT_CH2;
                *timer_channel = TIM_CHANNEL_2;
                return true;
            case HAL_TIM_ACTIVE_CHANNEL_3:
                *input_channel = RC_INPUT_CH5;
                *timer_channel = TIM_CHANNEL_3;
                return true;
            case HAL_TIM_ACTIVE_CHANNEL_4:
                *input_channel = RC_INPUT_CH6;
                *timer_channel = TIM_CHANNEL_4;
                return true;
            default:
                return false;
        }
    }
    if ((timer->Instance == TIM3) &&
        (timer->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
    {
        *input_channel = RC_INPUT_CH7;
        *timer_channel = TIM_CHANNEL_1;
        return true;
    }
    return false;
}

static void publish_sample(rc_input_channel_t channel,
                           uint16_t pulse_us,
                           uint32_t timestamp_ms,
                           bool capture_lost)
{
    /* TIM3 can preempt TIM8. Serialize both producers from slot lookup to
       publication, including the shared drop count. */
    const uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    const uint8_t head = queue_head;
    const uint8_t next_head =
        (uint8_t)((head + 1u) % RC_CAPTURE_QUEUE_CAPACITY);

    shared_samples[channel].pulse_us = pulse_us;
    shared_samples[channel].timestamp_ms = timestamp_ms;
    ++shared_samples[channel].sequence;

    const bool queued = channel != RC_INPUT_CH5 || inspection_channel_queued;
    if (queued && (capture_lost || (next_head == queue_tail)))
    {
        ++dropped_samples;
    }
    else if (queued)
    {
        sample_queue[head].channel = channel;
        sample_queue[head].pulse_us = pulse_us;
        sample_queue[head].timestamp_ms = timestamp_ms;
        __DMB();
        queue_head = next_head;
    }
    __DMB();
    if (interrupt_state == 0u)
    {
        __enable_irq();
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *timer)
{
    rc_input_channel_t input_channel;
    uint32_t timer_channel;
    uint16_t captured_count;
    rc_capture_edge_state_t *edge_state;
    uint32_t now_ms;
    uint32_t overcapture_flag;

    if ((timer == NULL) ||
        !map_active_channel(timer, &input_channel, &timer_channel))
    {
        return;
    }

    captured_count = (uint16_t)HAL_TIM_ReadCapturedValue(timer, timer_channel);
    now_ms = HAL_GetTick();
    edge_state = &edge_states[input_channel];
    /* F1 CCxOF bits are consecutive; HAL channel constants are 0,4,8,12. */
    overcapture_flag = TIM_FLAG_CC1OF << (timer_channel / 4u);
    if (__HAL_TIM_GET_FLAG(timer, overcapture_flag))
    {
        __HAL_TIM_CLEAR_FLAG(timer, overcapture_flag);
        edge_state->waiting_for_falling = false;
        set_capture_polarity(timer, timer_channel,
                             TIM_INPUTCHANNELPOLARITY_RISING);
        publish_sample(input_channel, 0u, now_ms, true);
        return;
    }
    if (!edge_state->waiting_for_falling)
    {
        edge_state->rising_count = captured_count;
        edge_state->rising_ms = now_ms;
        edge_state->waiting_for_falling = true;
        set_capture_polarity(timer, timer_channel,
                             TIM_INPUTCHANNELPOLARITY_FALLING);
        return;
    }

    edge_state->waiting_for_falling = false;
    set_capture_polarity(timer, timer_channel,
                         TIM_INPUTCHANNELPOLARITY_RISING);
    publish_sample(input_channel,
                   (uint32_t)(now_ms - edge_state->rising_ms) <=
                       ((RC_INPUT_PULSE_MAX_US + 999u) / 1000u) ?
                       rc_capture_elapsed_ticks(edge_state->rising_count,
                                                captured_count) : 0u,
                   now_ms, false);
}
