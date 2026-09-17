#include "board_status_outputs.h"

#include <stdbool.h>
#include <stdio.h>

static unsigned tests_run;

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
            return false;                                                       \
        }                                                                       \
    } while (0)

#define RUN(test)                                                               \
    do                                                                          \
    {                                                                           \
        ++tests_run;                                                            \
        if (!(test)())                                                          \
        {                                                                       \
            return 1;                                                           \
        }                                                                       \
    } while (0)

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    (void)pin;
    ++port->writes;
    port->last_state = state;
}

static bool test_emergency_beep_is_non_blocking_and_turns_off_on_exit(void)
{
    board_status_outputs_t outputs;
    GPIO_TypeDef beep_port = {0};

    CHECK(board_status_outputs_init(&outputs, &beep_port, 0x0100u, true, 200u));
    CHECK(beep_port.last_state == GPIO_PIN_RESET);
    board_status_outputs_poll(&outputs, APP_STATE_EMERGENCY, 10u);
    CHECK(beep_port.last_state == GPIO_PIN_SET);
    board_status_outputs_poll(&outputs, APP_STATE_EMERGENCY, 209u);
    CHECK(beep_port.last_state == GPIO_PIN_SET);
    board_status_outputs_poll(&outputs, APP_STATE_EMERGENCY, 210u);
    CHECK(beep_port.last_state == GPIO_PIN_RESET);
    board_status_outputs_poll(&outputs, APP_STATE_STANDBY, 211u);
    CHECK(beep_port.last_state == GPIO_PIN_RESET);
    return true;
}

static bool test_emergency_beep_deadline_handles_tick_wrap(void)
{
    board_status_outputs_t outputs;
    GPIO_TypeDef beep_port = {0};
    const uint32_t started = UINT32_MAX - 99u;

    CHECK(board_status_outputs_init(&outputs, &beep_port, 0x0100u, true, 200u));
    board_status_outputs_poll(&outputs, APP_STATE_EMERGENCY, started);
    CHECK(beep_port.last_state == GPIO_PIN_SET);
    board_status_outputs_poll(&outputs, APP_STATE_EMERGENCY, started + 199u);
    CHECK(beep_port.last_state == GPIO_PIN_SET);
    board_status_outputs_poll(&outputs, APP_STATE_EMERGENCY, started + 200u);
    CHECK(beep_port.last_state == GPIO_PIN_RESET);
    return true;
}

static bool test_stop_forces_beeper_off(void)
{
    board_status_outputs_t outputs;
    GPIO_TypeDef beep_port = {0};

    CHECK(board_status_outputs_init(&outputs, &beep_port, 0x0100u, true, 200u));
    board_status_outputs_poll(&outputs, APP_STATE_EMERGENCY, 10u);
    CHECK(beep_port.last_state == GPIO_PIN_SET);
    board_status_outputs_stop(&outputs);
    CHECK(beep_port.last_state == GPIO_PIN_RESET);
    CHECK(!outputs.emergency_active && !outputs.beep_on);
    return true;
}

int main(void)
{
    RUN(test_emergency_beep_is_non_blocking_and_turns_off_on_exit);
    RUN(test_emergency_beep_deadline_handles_tick_wrap);
    RUN(test_stop_forces_beeper_off);
    printf("all %u board_status_outputs tests passed\n", tests_run);
    return 0;
}
