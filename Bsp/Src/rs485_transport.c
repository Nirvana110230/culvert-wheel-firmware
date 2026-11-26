#include "rs485_transport.h"

#include <string.h>

static void enter_state(rs485_transport_t *transport,
                        rs485_state_t state,
                        uint32_t now_ms)
{
    transport->state = state;
    transport->state_enter_ms = now_ms;
}

static bool deadline_reached(uint32_t now_ms, uint32_t started_ms, uint32_t limit_ms)
{
    return (uint32_t)(now_ms - started_ms) >= limit_ms;
}

static bool set_receive_direction(rs485_transport_t *transport)
{
    if ((transport->port.set_de == NULL) ||
        !transport->port.set_de(false, transport->port.context))
    {
        ++transport->errors.de_restore_failures;
        return false;
    }
    return true;
}

static void enter_terminal_error(rs485_transport_t *transport,
                                 rs485_state_t terminal_state,
                                 uint32_t now_ms)
{
    (void)set_receive_direction(transport);
    if (terminal_state == RS485_TIMEOUT)
    {
        ++transport->errors.timeouts;
    }
    else
    {
        ++transport->errors.transaction_errors;
    }
    enter_state(transport, terminal_state, now_ms);
}

void rs485_transport_init(rs485_transport_t *transport,
                          rs485_port_t port,
                          rs485_timeouts_t timeouts)
{
    if (transport == NULL)
    {
        return;
    }
    (void)memset(transport, 0, sizeof(*transport));
    transport->port = port;
    transport->timeouts = timeouts;
    transport->state = RS485_IDLE;
    if (!set_receive_direction(transport))
    {
        enter_state(transport, RS485_ERROR, 0u);
    }
}

bool rs485_transport_begin(rs485_transport_t *transport,
                           const uint8_t *request,
                           size_t request_length,
                           uint32_t now_ms)
{
    if ((transport == NULL) || (transport->state != RS485_IDLE) ||
        (request == NULL) || (request_length == 0u) ||
        (request_length > sizeof(transport->request)))
    {
        return false;
    }
    (void)memcpy(transport->request, request, request_length);
    transport->request_length = request_length;
    enter_state(transport, RS485_SWITCH_TO_TX, now_ms);
    return true;
}

void rs485_transport_poll(rs485_transport_t *transport, uint32_t now_ms)
{
    if (transport == NULL)
    {
        return;
    }

    switch (transport->state)
    {
        case RS485_SWITCH_TO_TX:
            if ((transport->port.set_de == NULL) ||
                !transport->port.set_de(true, transport->port.context) ||
                (transport->port.start_dma == NULL) ||
                !transport->port.start_dma(transport->request,
                                           transport->request_length,
                                           transport->port.context))
            {
                enter_terminal_error(transport, RS485_ERROR, now_ms);
            }
            else
            {
                enter_state(transport, RS485_DMA_TX, now_ms);
            }
            break;
        case RS485_DMA_TX:
            if (deadline_reached(now_ms,
                                 transport->state_enter_ms,
                                 transport->timeouts.dma_ms))
            {
                enter_terminal_error(transport, RS485_TIMEOUT, now_ms);
            }
            break;
        case RS485_WAIT_UART_TC:
            if ((transport->port.uart_tc != NULL) &&
                transport->port.uart_tc(transport->port.context))
            {
                enter_state(transport, RS485_SWITCH_TO_RX, now_ms);
            }
            else if (deadline_reached(now_ms,
                                      transport->state_enter_ms,
                                      transport->timeouts.uart_tc_ms))
            {
                enter_terminal_error(transport, RS485_TIMEOUT, now_ms);
            }
            break;
        case RS485_SWITCH_TO_RX:
            if (set_receive_direction(transport))
            {
                enter_state(transport, RS485_WAIT_RESPONSE, now_ms);
            }
            else
            {
                ++transport->errors.transaction_errors;
                enter_state(transport, RS485_ERROR, now_ms);
            }
            break;
        case RS485_WAIT_RESPONSE:
            if (deadline_reached(now_ms,
                                 transport->state_enter_ms,
                                 transport->timeouts.response_ms))
            {
                enter_terminal_error(transport, RS485_TIMEOUT, now_ms);
            }
            break;
        case RS485_IDLE:
        case RS485_COMPLETE:
        case RS485_TIMEOUT:
        case RS485_ERROR:
        default:
            break;
    }
}

void rs485_transport_wait_for_bus(rs485_transport_t *transport, uint32_t now_ms)
{
    if ((transport != NULL) && (transport->state == RS485_SWITCH_TO_TX) &&
        deadline_reached(now_ms, transport->state_enter_ms,
                         transport->timeouts.response_ms))
    {
        enter_terminal_error(transport, RS485_TIMEOUT, now_ms);
    }
}

bool rs485_transport_dma_complete(rs485_transport_t *transport, uint32_t now_ms)
{
    if ((transport == NULL) || (transport->state != RS485_DMA_TX))
    {
        if (transport != NULL)
        {
            ++transport->errors.invalid_callbacks;
        }
        return false;
    }
    enter_state(transport, RS485_WAIT_UART_TC, now_ms);
    return true;
}

bool rs485_transport_uart_error(rs485_transport_t *transport, uint32_t now_ms)
{
    if ((transport == NULL) || (transport->state == RS485_IDLE) ||
        (transport->state == RS485_COMPLETE) || (transport->state == RS485_TIMEOUT) ||
        (transport->state == RS485_ERROR))
    {
        if (transport != NULL)
        {
            ++transport->errors.invalid_callbacks;
        }
        return false;
    }
    enter_terminal_error(transport, RS485_ERROR, now_ms);
    return true;
}

bool rs485_transport_response_complete(rs485_transport_t *transport,
                                       uint32_t now_ms)
{
    if ((transport == NULL) || (transport->state != RS485_WAIT_RESPONSE))
    {
        if (transport != NULL)
        {
            ++transport->errors.invalid_callbacks;
        }
        return false;
    }
    enter_state(transport, RS485_COMPLETE, now_ms);
    return true;
}

bool rs485_transport_abort(rs485_transport_t *transport, uint32_t now_ms)
{
    if ((transport == NULL) || (transport->state == RS485_IDLE) ||
        (transport->state == RS485_COMPLETE) || (transport->state == RS485_TIMEOUT) ||
        (transport->state == RS485_ERROR))
    {
        return false;
    }
    ++transport->errors.aborts;
    enter_terminal_error(transport, RS485_ERROR, now_ms);
    return true;
}

bool rs485_transport_acknowledge(rs485_transport_t *transport, uint32_t now_ms)
{
    if ((transport == NULL) ||
        ((transport->state != RS485_COMPLETE) &&
         (transport->state != RS485_TIMEOUT) &&
         (transport->state != RS485_ERROR)))
    {
        return false;
    }
    if (!set_receive_direction(transport))
    {
        enter_state(transport, RS485_ERROR, now_ms);
        return false;
    }
    transport->request_length = 0u;
    enter_state(transport, RS485_IDLE, now_ms);
    return true;
}
