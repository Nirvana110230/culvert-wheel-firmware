#ifndef BOARD_STATUS_OUTPUTS_H
#define BOARD_STATUS_OUTPUTS_H

#include "app_state_machine.h"
#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    GPIO_TypeDef *beep_port;
    uint16_t beep_pin;
    uint32_t half_period_ms;
    uint32_t next_toggle_ms;
    bool active_high;
    bool emergency_active;
    bool beep_on;
    bool initialized;
} board_status_outputs_t;

/* Initializes the output in the inactive state. */
bool board_status_outputs_init(board_status_outputs_t *outputs,
                               GPIO_TypeDef *beep_port,
                               uint16_t beep_pin,
                               bool active_high,
                               uint32_t half_period_ms);

/* Advances the non-blocking emergency indication. */
void board_status_outputs_poll(board_status_outputs_t *outputs,
                               app_state_t state,
                               uint32_t now_ms);

/* Forces the beeper to its inactive level. */
void board_status_outputs_stop(board_status_outputs_t *outputs);

#endif
