#include "rc_capture.h"
#include "tim.h"

#include <stdio.h>
#include <stdlib.h>

TIM_HandleTypeDef htim3 = {.Instance = TIM3};
TIM_HandleTypeDef htim8 = {.Instance = TIM8};
static uint32_t tick, interrupt_mask;
static unsigned starts, stops;

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); \
    exit(EXIT_FAILURE); } } while (0)

uint32_t __get_PRIMASK(void) { return interrupt_mask; }
void __disable_irq(void) { interrupt_mask = 1u; }
void __enable_irq(void) { interrupt_mask = 0u; }
uint32_t HAL_GetTick(void) { return tick; }

HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *timer, uint32_t channel)
{
    CHECK(timer->polarity[channel / 4u] == TIM_INPUTCHANNELPOLARITY_RISING);
    ++starts;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *timer, uint32_t channel)
{
    (void)timer;
    (void)channel;
    ++stops;
    return HAL_OK;
}

uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *timer, uint32_t channel)
{
    CHECK((1u << (channel / 4u)) == timer->Channel);
    return timer->count;
}

static void edge(TIM_HandleTypeDef *timer, unsigned channel,
                 uint16_t counter, uint32_t time_ms)
{
    tick = time_ms;
    timer->Channel = 1u << channel;
    timer->count = counter;
    HAL_TIM_IC_CaptureCallback(timer);
}

int main(void)
{
    rc_capture_snapshot_t snapshot;
    rc_capture_sample_t sample;
    CHECK(rc_capture_start(true));
    CHECK(starts == 5u);
    for (unsigned i = 0u; i < RC_INPUT_CHANNEL_COUNT; ++i)
    {
        TIM_HandleTypeDef *timer = i == RC_INPUT_CH7 ? &htim3 : &htim8;
        unsigned ch = i == RC_INPUT_CH7 ? 0u : i;
        edge(timer, ch, 65000u, UINT32_MAX);
        CHECK(timer->polarity[ch] == TIM_INPUTCHANNELPOLARITY_FALLING);
        edge(timer, ch, 964u, 1u);
        CHECK(timer->polarity[ch] == TIM_INPUTCHANNELPOLARITY_RISING);
        CHECK(rc_capture_take_sample(&sample));
        CHECK((unsigned)sample.channel == i && sample.pulse_us == 1500u);
        CHECK(sample.timestamp_ms == 1u);
    }
    edge(&htim3, 0u, 0u, 10u);
    edge(&htim3, 0u, 1000u, 77u);
    CHECK(rc_capture_take_sample(&sample) && sample.pulse_us == 0u);

    /* One lost edge on TIM8 CH2 must reset polarity and be observable. */
    edge(&htim8, 1u, 0u, 80u);
    htim8.flags = TIM_FLAG_CC1OF << 1u;
    edge(&htim8, 1u, 1500u, 82u);
    CHECK(htim8.flags == 0u);
    CHECK(htim8.polarity[1] == TIM_INPUTCHANNELPOLARITY_RISING);
    CHECK(!rc_capture_take_sample(&sample));
    rc_capture_read_snapshot(&snapshot);
    CHECK(snapshot.dropped_samples == 1u);

    interrupt_mask = 1u;
    for (unsigned i = 0u; i < 16u; ++i)
    {
        edge(&htim3, 0u, 0u, 100u + i * 20u);
        edge(&htim3, 0u, 2000u, 102u + i * 20u);
    }
    rc_capture_read_snapshot(&snapshot);
    CHECK(snapshot.dropped_samples == 2u);
    CHECK(interrupt_mask == 1u);
    for (unsigned i = 0u; i < 15u; ++i)
    {
        CHECK(rc_capture_take_sample(&sample));
        CHECK(sample.timestamp_ms == 102u + i * 20u);
    }
    CHECK(!rc_capture_take_sample(&sample) && interrupt_mask == 1u);
    rc_capture_stop();
    CHECK(stops == 5u);
    CHECK(rc_capture_start(false));
    for (unsigned i = 0u; i < 20u; ++i)
    {
        edge(&htim8, 2u, 0u, 100u + i * 20u);
        edge(&htim8, 2u, 1500u, 102u + i * 20u);
    }
    rc_capture_read_snapshot(&snapshot);
    CHECK(snapshot.channels[RC_INPUT_CH5].sequence == 20u);
    CHECK(snapshot.dropped_samples == 0u && !rc_capture_take_sample(&sample));
    rc_capture_stop();
    puts("rc_capture cases passed");
    return EXIT_SUCCESS;
}
