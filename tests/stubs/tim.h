#ifndef TEST_TIM_H
#define TEST_TIM_H

#include "stm32f1xx_hal.h"

typedef struct
{
    uintptr_t Instance;
    uint32_t Channel;
    uint32_t flags;
    uint32_t count;
    uint32_t polarity[4];
} TIM_HandleTypeDef;

#define TIM3 3u
#define TIM8 8u
#define TIM_CHANNEL_1 0u
#define TIM_CHANNEL_2 4u
#define TIM_CHANNEL_3 8u
#define TIM_CHANNEL_4 12u
#define HAL_TIM_ACTIVE_CHANNEL_1 1u
#define HAL_TIM_ACTIVE_CHANNEL_2 2u
#define HAL_TIM_ACTIVE_CHANNEL_3 4u
#define HAL_TIM_ACTIVE_CHANNEL_4 8u
#define TIM_FLAG_CC1OF 0x0200u
#define TIM_INPUTCHANNELPOLARITY_RISING 0u
#define TIM_INPUTCHANNELPOLARITY_FALLING 1u
#define __DMB() ((void)0)
#define __HAL_TIM_SET_CAPTUREPOLARITY(timer, channel, value) \
    ((timer)->polarity[(channel) / 4u] = (value))
#define __HAL_TIM_GET_FLAG(timer, bit) (((timer)->flags & (bit)) != 0u)
#define __HAL_TIM_CLEAR_FLAG(timer, bit) ((timer)->flags &= ~(bit))

extern TIM_HandleTypeDef htim8, htim3;
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *timer, uint32_t channel);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *timer, uint32_t channel);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *timer);
#endif
