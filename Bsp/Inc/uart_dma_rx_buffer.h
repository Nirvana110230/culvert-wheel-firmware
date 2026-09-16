#ifndef UART_DMA_RX_BUFFER_H
#define UART_DMA_RX_BUFFER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UART_DMA_RX_RING_SIZE 64u

typedef struct
{
    uint32_t publish_events;
    uint32_t invalid_positions;
    uint32_t overrun_events;
    uint32_t dropped_bytes;
} uart_dma_rx_buffer_stats_t;

typedef struct
{
    uint8_t ring[UART_DMA_RX_RING_SIZE];
    /* ISR is the sole head writer; foreground is the sole tail writer. */
    volatile uint16_t head;
    volatile uint16_t tail;
    uint16_t dma_position;
    volatile bool overflow_pending;
    volatile uart_dma_rx_buffer_stats_t stats;
} uart_dma_rx_buffer_t;

void uart_dma_rx_buffer_init(uart_dma_rx_buffer_t *buffer);

/* Call only while the producer DMA/IRQ is stopped. Statistics are preserved. */
void uart_dma_rx_buffer_reset_stream(uart_dma_rx_buffer_t *buffer);

/* ISR producer: Size is a DMA write position, not a new-byte count. */
bool uart_dma_rx_buffer_publish(uart_dma_rx_buffer_t *buffer,
                                const uint8_t *dma_data,
                                size_t dma_capacity,
                                uint16_t position,
                                bool transfer_complete);

/* Foreground consumer. */
bool uart_dma_rx_buffer_take(uart_dma_rx_buffer_t *buffer, uint8_t *byte);
size_t uart_dma_rx_buffer_count(const uart_dma_rx_buffer_t *buffer);
void uart_dma_rx_buffer_discard(uart_dma_rx_buffer_t *buffer);
bool uart_dma_rx_buffer_take_overflow(uart_dma_rx_buffer_t *buffer);
uart_dma_rx_buffer_stats_t uart_dma_rx_buffer_get_stats(
    const uart_dma_rx_buffer_t *buffer);

#endif
