#include "uart_dma_rx_buffer.h"

#include <string.h>

static uint16_t ring_next(uint16_t position)
{
    return (uint16_t)((position + 1u) % UART_DMA_RX_RING_SIZE);
}

static bool publish_byte(uart_dma_rx_buffer_t *buffer, uint8_t byte)
{
    const uint16_t head = buffer->head;
    const uint16_t next = ring_next(head);

    if (next == buffer->tail)
    {
        ++buffer->stats.dropped_bytes;
        buffer->overflow_pending = true;
        return false;
    }
    buffer->ring[head] = byte;
    buffer->head = next;
    return true;
}

static bool publish_range(uart_dma_rx_buffer_t *buffer,
                          const uint8_t *dma_data,
                          size_t begin,
                          size_t end)
{
    bool complete = true;
    size_t index;

    for (index = begin; index < end; ++index)
    {
        if (!publish_byte(buffer, dma_data[index]))
        {
            complete = false;
        }
    }
    return complete;
}

void uart_dma_rx_buffer_init(uart_dma_rx_buffer_t *buffer)
{
    if (buffer != NULL)
    {
        (void)memset(buffer, 0, sizeof(*buffer));
    }
}

void uart_dma_rx_buffer_reset_stream(uart_dma_rx_buffer_t *buffer)
{
    if (buffer != NULL)
    {
        buffer->head = 0u;
        buffer->tail = 0u;
        buffer->dma_position = 0u;
        buffer->overflow_pending = false;
    }
}

bool uart_dma_rx_buffer_publish(uart_dma_rx_buffer_t *buffer,
                                const uint8_t *dma_data,
                                size_t dma_capacity,
                                uint16_t position,
                                bool transfer_complete)
{
    bool complete = true;
    uint16_t current;
    const uint32_t dropped_before =
        (buffer == NULL) ? 0u : buffer->stats.dropped_bytes;

    if ((buffer == NULL) || (dma_data == NULL) || (dma_capacity == 0u) ||
        (dma_capacity > UINT16_MAX) || ((size_t)position > dma_capacity) ||
        (transfer_complete && ((size_t)position != dma_capacity)))
    {
        if (buffer != NULL)
        {
            ++buffer->stats.invalid_positions;
        }
        return false;
    }

    ++buffer->stats.publish_events;
    current = ((size_t)position == dma_capacity) ? 0u : position;
    if (transfer_complete)
    {
        complete = publish_range(buffer,
                                 dma_data,
                                 buffer->dma_position,
                                 dma_capacity);
    }
    else if (current > buffer->dma_position)
    {
        complete = publish_range(buffer,
                                 dma_data,
                                 buffer->dma_position,
                                 current);
    }
    else if (current < buffer->dma_position)
    {
        complete = publish_range(buffer,
                                 dma_data,
                                 buffer->dma_position,
                                 dma_capacity);
        if (!publish_range(buffer, dma_data, 0u, current))
        {
            complete = false;
        }
    }
    else
    {
        /* Duplicate IDLE position: no newly published bytes. */
    }
    buffer->dma_position = current;

    if (buffer->stats.dropped_bytes != dropped_before)
    {
        ++buffer->stats.overrun_events;
    }
    return complete;
}

bool uart_dma_rx_buffer_take(uart_dma_rx_buffer_t *buffer, uint8_t *byte)
{
    uint16_t tail;

    if ((buffer == NULL) || (byte == NULL) || (buffer->tail == buffer->head))
    {
        return false;
    }
    tail = buffer->tail;
    *byte = buffer->ring[tail];
    buffer->tail = ring_next(tail);
    return true;
}

size_t uart_dma_rx_buffer_count(const uart_dma_rx_buffer_t *buffer)
{
    const uint16_t head = (buffer == NULL) ? 0u : buffer->head;
    const uint16_t tail = (buffer == NULL) ? 0u : buffer->tail;

    if (buffer == NULL)
    {
        return 0u;
    }
    if (head >= tail)
    {
        return (size_t)(head - tail);
    }
    return (size_t)(UART_DMA_RX_RING_SIZE - tail + head);
}

void uart_dma_rx_buffer_discard(uart_dma_rx_buffer_t *buffer)
{
    if (buffer != NULL)
    {
        buffer->tail = buffer->head;
    }
}

bool uart_dma_rx_buffer_take_overflow(uart_dma_rx_buffer_t *buffer)
{
    if ((buffer == NULL) || !buffer->overflow_pending)
    {
        return false;
    }
    buffer->overflow_pending = false;
    return true;
}

uart_dma_rx_buffer_stats_t uart_dma_rx_buffer_get_stats(
    const uart_dma_rx_buffer_t *buffer)
{
    uart_dma_rx_buffer_stats_t stats = {0};

    if (buffer != NULL)
    {
        stats.publish_events = buffer->stats.publish_events;
        stats.invalid_positions = buffer->stats.invalid_positions;
        stats.overrun_events = buffer->stats.overrun_events;
        stats.dropped_bytes = buffer->stats.dropped_bytes;
    }
    return stats;
}
