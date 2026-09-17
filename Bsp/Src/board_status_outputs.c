#include "board_status_outputs.h"

#include <stddef.h>
#include <string.h>

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (uint32_t)(now_ms - deadline_ms) < 0x80000000u;
}

static void write_beep(board_status_outputs_t *outputs, bool on)
{
    const bool high = outputs->active_high ? on : !on;

    HAL_GPIO_WritePin(outputs->beep_port,
                      outputs->beep_pin,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    outputs->beep_on = on;
}

bool board_status_outputs_init(board_status_outputs_t *outputs,
                               GPIO_TypeDef *beep_port,
                               uint16_t beep_pin,
                               bool active_high,
                               uint32_t half_period_ms)
{
    if ((outputs == NULL) || (beep_port == NULL) ||
        (beep_pin == 0u) || (half_period_ms == 0u))
    {
        return false;
    }
    (void)memset(outputs, 0, sizeof(*outputs));
    outputs->beep_port = beep_port;
    outputs->beep_pin = beep_pin;
    outputs->active_high = active_high;
    outputs->half_period_ms = half_period_ms;
    outputs->initialized = true;
    write_beep(outputs, false);
    return true;
}

void board_status_outputs_poll(board_status_outputs_t *outputs,
                               app_state_t state,
                               uint32_t now_ms)
{
    if ((outputs == NULL) || !outputs->initialized)
    {
        return;
    }
    if (state != APP_STATE_EMERGENCY)
    {
        if (outputs->emergency_active || outputs->beep_on)
        {
            write_beep(outputs, false);
        }
        outputs->emergency_active = false;
        return;
    }
    if (!outputs->emergency_active)
    {
        outputs->emergency_active = true;
        write_beep(outputs, true);
        outputs->next_toggle_ms = now_ms + outputs->half_period_ms;
        return;
    }
    if (time_reached(now_ms, outputs->next_toggle_ms))
    {
        write_beep(outputs, !outputs->beep_on);
        outputs->next_toggle_ms = now_ms + outputs->half_period_ms;
    }
}

void board_status_outputs_stop(board_status_outputs_t *outputs)
{
    if ((outputs == NULL) || !outputs->initialized)
    {
        return;
    }
    write_beep(outputs, false);
    outputs->emergency_active = false;
}
