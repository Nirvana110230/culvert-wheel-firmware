#include "jetson_uart_transport.h"

#include "usart.h"

#include <string.h>

static jetson_uart_transport_t *volatile active_transport;

static uint16_t ring_next(uint16_t position)
{
    return (uint16_t)((position + 1u) % JETSON_UART_SOFTWARE_RING_SIZE);
}

static void publish_byte_from_isr(jetson_uart_transport_t *transport, uint8_t byte)
{
    const uint16_t head = transport->ring_head;
    const uint16_t next = ring_next(head);

    if (next == transport->ring_tail)
    {
        ++transport->stats.dropped_bytes;
        return;
    }
    transport->rx_ring[head] = byte;
    transport->ring_head = next;
}

static void publish_range_from_isr(jetson_uart_transport_t *transport,
                                   uint16_t begin,
                                   uint16_t end)
{
    uint16_t index;
    const uint32_t dropped_before = transport->stats.dropped_bytes;

    for (index = begin; index < end; ++index)
    {
        publish_byte_from_isr(transport, transport->rx_dma[index]);
    }
    if (transport->stats.dropped_bytes != dropped_before)
    {
        ++transport->stats.software_overruns;
    }
}

static void clear_receive_state(jetson_uart_transport_t *transport)
{
    transport->dma_position = 0u;
    transport->ring_head = 0u;
    transport->ring_tail = 0u;
    jetson_protocol_reset_partial_frame(transport->parser);
}

static bool start_receive_dma(jetson_uart_transport_t *transport)
{
    transport->state = JETSON_UART_RUNNING;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                     transport->rx_dma,
                                     (uint16_t)sizeof(transport->rx_dma)) != HAL_OK)
    {
        ++transport->stats.restart_failures;
        transport->state = JETSON_UART_STOPPED;
        return false;
    }
    return true;
}

bool jetson_uart_transport_start(jetson_uart_transport_t *transport,
                                 jetson_protocol_t *parser)
{
    if ((transport == NULL) || (parser == NULL) || (active_transport != NULL))
    {
        return false;
    }

    (void)memset(transport, 0, sizeof(*transport));
    transport->parser = parser;
    active_transport = transport;
    if (!start_receive_dma(transport))
    {
        active_transport = NULL;
        return false;
    }
    return true;
}

bool jetson_uart_transport_reset(jetson_uart_transport_t *transport)
{
    if ((transport == NULL) || (transport->parser == NULL) ||
        (active_transport != transport))
    {
        return false;
    }

    transport->state = JETSON_UART_STOPPED;
    if (HAL_UART_DMAStop(&huart1) != HAL_OK)
    {
        ++transport->stats.dma_stop_failures;
        clear_receive_state(transport);
        return false;
    }
    clear_receive_state(transport);
    return start_receive_dma(transport);
}

void jetson_uart_transport_on_rx_event(jetson_uart_transport_t *transport,
                                       uint16_t dma_position)
{
    uint16_t previous;

    if ((transport == NULL) || (transport->state != JETSON_UART_RUNNING))
    {
        return;
    }
    ++transport->stats.rx_events;
    if (dma_position > JETSON_UART_DMA_BUFFER_SIZE)
    {
        ++transport->stats.invalid_positions;
        return;
    }

    previous = transport->dma_position;
    if (dma_position == JETSON_UART_DMA_BUFFER_SIZE)
    {
        publish_range_from_isr(transport, previous, JETSON_UART_DMA_BUFFER_SIZE);
        transport->dma_position = 0u;
    }
    else if (dma_position > previous)
    {
        publish_range_from_isr(transport, previous, dma_position);
        transport->dma_position = dma_position;
    }
    else if (dma_position < previous)
    {
        publish_range_from_isr(transport, previous, JETSON_UART_DMA_BUFFER_SIZE);
        publish_range_from_isr(transport, 0u, dma_position);
        transport->dma_position = dma_position;
    }
    else
    {
        /* Duplicate HT/TC/IDLE position: no new bytes to publish. */
    }
}

void jetson_uart_transport_on_uart_error(jetson_uart_transport_t *transport)
{
    if ((transport != NULL) && (transport->state == JETSON_UART_RUNNING))
    {
        ++transport->stats.uart_errors;
        transport->state = JETSON_UART_RECOVERY_PENDING;
    }
}

void jetson_uart_transport_poll(jetson_uart_transport_t *transport, uint32_t now_ms)
{
    if ((transport == NULL) || (transport->parser == NULL))
    {
        return;
    }

    if (transport->state == JETSON_UART_RECOVERY_PENDING)
    {
        (void)jetson_uart_transport_reset(transport);
    }

    while ((transport->state == JETSON_UART_RUNNING) &&
           (transport->ring_tail != transport->ring_head))
    {
        const uint16_t tail = transport->ring_tail;
        const uint8_t byte = transport->rx_ring[tail];

        transport->ring_tail = ring_next(tail);
        jetson_protocol_feed(transport->parser, &byte, 1u, now_ms);
    }
    jetson_protocol_poll(transport->parser, now_ms);
}

bool jetson_uart_transport_send_async(jetson_uart_transport_t *transport,
                                      const uint8_t *data,
                                      size_t length)
{
    (void)transport;
    (void)data;
    (void)length;
    return false;
}

void jetson_uart_transport_dispatch_rx_event(uint16_t dma_position)
{
    if (active_transport != NULL)
    {
        jetson_uart_transport_on_rx_event(active_transport, dma_position);
    }
}

void jetson_uart_transport_dispatch_uart_error(void)
{
    if (active_transport != NULL)
    {
        jetson_uart_transport_on_uart_error(active_transport);
    }
}
