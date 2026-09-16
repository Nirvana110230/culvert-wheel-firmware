#include "rs485_transport.h"

#include <limits.h>
#include <stdio.h>

typedef struct
{
    bool de;
    bool set_tx_ok;
    bool set_rx_ok;
    bool dma_ok;
    bool tc;
} mock_port_t;

static unsigned tests_run;

#define CHECK(condition)                                                                    \
    do                                                                                      \
    {                                                                                       \
        if (!(condition))                                                                   \
        {                                                                                   \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);               \
            return false;                                                                   \
        }                                                                                   \
    } while (0)

#define RUN(test)                                                                           \
    do                                                                                      \
    {                                                                                       \
        if (!(test()))                                                                      \
        {                                                                                   \
            return 1;                                                                       \
        }                                                                                   \
        ++tests_run;                                                                        \
    } while (0)

static bool set_de(bool transmit, void *context)
{
    mock_port_t *mock = context;

    if ((transmit && !mock->set_tx_ok) || (!transmit && !mock->set_rx_ok))
    {
        return false;
    }
    mock->de = transmit;
    return true;
}

static bool start_dma(const uint8_t *data, size_t length, void *context)
{
    (void)data;
    (void)length;
    return ((mock_port_t *)context)->dma_ok;
}

static bool uart_tc(void *context)
{
    return ((mock_port_t *)context)->tc;
}

static void setup(rs485_transport_t *transport, mock_port_t *mock)
{
    const rs485_port_t port = {set_de, start_dma, uart_tc, mock};
    const rs485_timeouts_t timeouts = {10u, 20u, 30u};

    *mock = (mock_port_t){false, true, true, true, false};
    rs485_transport_init(transport, port, timeouts);
}

static bool reach_dma(rs485_transport_t *transport, mock_port_t *mock, uint32_t now_ms)
{
    static const uint8_t request[] = {0x01, 0x03};

    setup(transport, mock);
    CHECK(rs485_transport_begin(transport, request, sizeof(request), now_ms));
    CHECK(transport->state == RS485_SWITCH_TO_TX);
    rs485_transport_poll(transport, now_ms);
    CHECK((transport->state == RS485_DMA_TX) && mock->de);
    return true;
}

static bool reach_wait_uart_tc(rs485_transport_t *transport,
                               mock_port_t *mock,
                               uint32_t now_ms)
{
    CHECK(reach_dma(transport, mock, now_ms));
    CHECK(rs485_transport_dma_complete(transport, now_ms + 1u));
    CHECK(transport->state == RS485_WAIT_UART_TC);
    return true;
}

static bool reach_switch_to_rx(rs485_transport_t *transport,
                               mock_port_t *mock,
                               uint32_t now_ms)
{
    CHECK(reach_wait_uart_tc(transport, mock, now_ms));
    mock->tc = true;
    rs485_transport_poll(transport, now_ms + 2u);
    CHECK(transport->state == RS485_SWITCH_TO_RX);
    return true;
}

static bool reach_wait_response(rs485_transport_t *transport,
                                mock_port_t *mock,
                                uint32_t now_ms)
{
    CHECK(reach_switch_to_rx(transport, mock, now_ms));
    rs485_transport_poll(transport, now_ms + 3u);
    CHECK((transport->state == RS485_WAIT_RESPONSE) && !mock->de);
    return true;
}

static bool test_normal_path_reaches_every_state_and_acknowledges(void)
{
    rs485_transport_t transport;
    mock_port_t mock;

    CHECK(reach_wait_response(&transport, &mock, 1u));
    CHECK(rs485_transport_response_complete(&transport, 5u));
    CHECK(transport.state == RS485_COMPLETE);
    CHECK(rs485_transport_acknowledge(&transport, 6u));
    CHECK((transport.state == RS485_IDLE) && !mock.de);
    return true;
}

static bool test_begin_only_succeeds_from_idle(void)
{
    static const uint8_t request = 1u;
    rs485_transport_t transport;
    mock_port_t mock;

    setup(&transport, &mock);
    CHECK(rs485_transport_begin(&transport, &request, 1u, 0u));
    CHECK(!rs485_transport_begin(&transport, &request, 1u, 0u));
    rs485_transport_poll(&transport, 0u);
    CHECK(!rs485_transport_begin(&transport, &request, 1u, 0u));
    return true;
}

static bool test_dma_timeout_at_exact_deadline(void)
{
    rs485_transport_t transport;
    mock_port_t mock;

    CHECK(reach_dma(&transport, &mock, 100u));
    rs485_transport_poll(&transport, 109u);
    CHECK(transport.state == RS485_DMA_TX);
    rs485_transport_poll(&transport, 110u);
    CHECK((transport.state == RS485_TIMEOUT) && !mock.de);
    return true;
}

static bool test_uart_tc_timeout_uses_its_own_entry_time(void)
{
    rs485_transport_t transport;
    mock_port_t mock;

    CHECK(reach_wait_uart_tc(&transport, &mock, 100u));
    rs485_transport_poll(&transport, 120u);
    CHECK(transport.state == RS485_WAIT_UART_TC);
    rs485_transport_poll(&transport, 121u);
    CHECK((transport.state == RS485_TIMEOUT) && !mock.de);
    return true;
}

static bool test_response_timeout_uses_its_own_entry_time(void)
{
    rs485_transport_t transport;
    mock_port_t mock;

    CHECK(reach_wait_response(&transport, &mock, 100u));
    rs485_transport_poll(&transport, 132u);
    CHECK(transport.state == RS485_WAIT_RESPONSE);
    rs485_transport_poll(&transport, 133u);
    CHECK((transport.state == RS485_TIMEOUT) && !mock.de);
    return true;
}

static bool test_timeout_handles_tick_wrap(void)
{
    rs485_transport_t transport;
    mock_port_t mock;
    const uint32_t started = UINT32_MAX - 4u;

    CHECK(reach_dma(&transport, &mock, started));
    rs485_transport_poll(&transport, started + 9u);
    CHECK(transport.state == RS485_DMA_TX);
    rs485_transport_poll(&transport, started + 10u);
    CHECK((transport.state == RS485_TIMEOUT) && !mock.de);
    return true;
}

static bool test_set_de_high_failure_enters_error(void)
{
    static const uint8_t request = 1u;
    rs485_transport_t transport;
    mock_port_t mock;

    setup(&transport, &mock);
    mock.set_tx_ok = false;
    CHECK(rs485_transport_begin(&transport, &request, 1u, 0u));
    rs485_transport_poll(&transport, 0u);
    CHECK((transport.state == RS485_ERROR) && !mock.de);
    return true;
}

static bool test_start_dma_failure_restores_receive_direction(void)
{
    static const uint8_t request = 1u;
    rs485_transport_t transport;
    mock_port_t mock;

    setup(&transport, &mock);
    mock.dma_ok = false;
    CHECK(rs485_transport_begin(&transport, &request, 1u, 0u));
    rs485_transport_poll(&transport, 0u);
    CHECK((transport.state == RS485_ERROR) && !mock.de);
    return true;
}

static bool test_set_de_low_failure_is_recorded(void)
{
    rs485_transport_t transport;
    mock_port_t mock;

    CHECK(reach_switch_to_rx(&transport, &mock, 0u));
    mock.set_rx_ok = false;
    rs485_transport_poll(&transport, 3u);
    CHECK(transport.state == RS485_ERROR);
    CHECK(mock.de);
    CHECK(transport.errors.de_restore_failures == 1u);
    return true;
}

static bool test_uart_error_is_state_checked_and_restores_rx(void)
{
    rs485_transport_t transport;
    mock_port_t mock;

    CHECK(reach_dma(&transport, &mock, 0u));
    CHECK(rs485_transport_uart_error(&transport, 1u));
    CHECK((transport.state == RS485_ERROR) && !mock.de);
    CHECK(!rs485_transport_uart_error(&transport, 2u));
    CHECK(transport.errors.invalid_callbacks == 1u);
    return true;
}

static bool test_abort_restores_rx_and_enters_error(void)
{
    rs485_transport_t transport;
    mock_port_t mock;

    CHECK(reach_dma(&transport, &mock, 0u));
    CHECK(rs485_transport_abort(&transport, 1u));
    CHECK((transport.state == RS485_ERROR) && !mock.de);
    CHECK(transport.errors.aborts == 1u);
    CHECK(!rs485_transport_abort(&transport, 2u));
    return true;
}

static bool test_terminal_state_rejects_begin_until_acknowledged(void)
{
    static const uint8_t request = 1u;
    rs485_transport_t transport;
    mock_port_t mock;

    CHECK(reach_dma(&transport, &mock, 0u));
    rs485_transport_poll(&transport, 10u);
    CHECK(transport.state == RS485_TIMEOUT);
    CHECK(!rs485_transport_begin(&transport, &request, 1u, 11u));
    CHECK(rs485_transport_acknowledge(&transport, 11u));
    CHECK(rs485_transport_begin(&transport, &request, 1u, 12u));
    return true;
}

static bool test_invalid_callbacks_do_not_change_state(void)
{
    rs485_transport_t transport;
    mock_port_t mock;

    setup(&transport, &mock);
    CHECK(!rs485_transport_dma_complete(&transport, 0u));
    CHECK(!rs485_transport_response_complete(&transport, 0u));
    CHECK(transport.state == RS485_IDLE);
    CHECK(transport.errors.invalid_callbacks == 2u);
    return true;
}

static bool test_request_capacity_and_null_inputs(void)
{
    uint8_t request[RS485_TRANSPORT_MAX_REQUEST + 1u] = {0u};
    rs485_transport_t transport;
    mock_port_t mock;

    setup(&transport, &mock);
    CHECK(!rs485_transport_begin(&transport, request, 0u, 0u));
    CHECK(!rs485_transport_begin(&transport, request, sizeof(request), 0u));
    CHECK(!rs485_transport_begin(NULL, request, 1u, 0u));
    return true;
}

int main(void)
{
    RUN(test_normal_path_reaches_every_state_and_acknowledges);
    RUN(test_begin_only_succeeds_from_idle);
    RUN(test_dma_timeout_at_exact_deadline);
    RUN(test_uart_tc_timeout_uses_its_own_entry_time);
    RUN(test_response_timeout_uses_its_own_entry_time);
    RUN(test_timeout_handles_tick_wrap);
    RUN(test_set_de_high_failure_enters_error);
    RUN(test_start_dma_failure_restores_receive_direction);
    RUN(test_set_de_low_failure_is_recorded);
    RUN(test_uart_error_is_state_checked_and_restores_rx);
    RUN(test_abort_restores_rx_and_enters_error);
    RUN(test_terminal_state_rejects_begin_until_acknowledged);
    RUN(test_invalid_callbacks_do_not_change_state);
    RUN(test_request_capacity_and_null_inputs);
    printf("all %u rs485_transport tests passed\n", tests_run);
    return 0;
}
