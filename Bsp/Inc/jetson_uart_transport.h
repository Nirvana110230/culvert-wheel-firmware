#ifndef JETSON_UART_TRANSPORT_H
#define JETSON_UART_TRANSPORT_H

#include "jetson_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define JETSON_UART_DMA_BUFFER_SIZE 128u
#define JETSON_UART_SOFTWARE_RING_SIZE 256u

typedef enum
{
    JETSON_UART_STOPPED = 0,
    JETSON_UART_RUNNING,
    JETSON_UART_RECOVERY_PENDING
} jetson_uart_state_t;

typedef struct
{
    volatile uint32_t rx_events;
    volatile uint32_t invalid_positions;
    volatile uint32_t software_overruns;
    volatile uint32_t dropped_bytes;
    volatile uint32_t uart_errors;
    volatile uint32_t tx_completions;
    uint32_t dma_stop_failures;
    uint32_t restart_failures;
    uint32_t tx_start_failures;
} jetson_uart_stats_t;

typedef struct
{
    jetson_protocol_t *parser;
    uint8_t rx_dma[JETSON_UART_DMA_BUFFER_SIZE];
    uint8_t rx_ring[JETSON_UART_SOFTWARE_RING_SIZE];
    uint8_t tx_buffer[JETSON_PROTOCOL_MAX_FRAME_SIZE];
    volatile uint16_t dma_position;
    volatile uint16_t ring_head;
    volatile uint16_t ring_tail;
    volatile jetson_uart_state_t state;
    volatile bool tx_complete_pending;
    bool tx_busy;
    jetson_uart_stats_t stats;
} jetson_uart_transport_t;

/* Starts USART1 circular DMA; transport and parser must outlive all callbacks. */
bool jetson_uart_transport_start(jetson_uart_transport_t *transport,
                                 jetson_protocol_t *parser);

/* Stops RX DMA, clears queued bytes, and releases the callback dispatcher. */
bool jetson_uart_transport_stop(jetson_uart_transport_t *transport);

/* Drains the ISR-owned byte ring and performs protocol parsing in main context. */
void jetson_uart_transport_poll(jetson_uart_transport_t *transport, uint32_t now_ms);

/* Stops, clears partial receive data, and restarts RX DMA. */
bool jetson_uart_transport_reset(jetson_uart_transport_t *transport);

/* ISR entry points used by the HAL callbacks; they never parse complete frames. */
void jetson_uart_transport_on_rx_event(jetson_uart_transport_t *transport,
                                       uint16_t dma_position);
void jetson_uart_transport_on_uart_error(jetson_uart_transport_t *transport);

/* Called only by the shared HAL callback dispatcher. */
void jetson_uart_transport_dispatch_rx_event(uint16_t dma_position);
void jetson_uart_transport_dispatch_uart_error(void);
void jetson_uart_transport_dispatch_tx_complete(void);

/* Copies one bounded frame and starts USART1 TX DMA; only one send may be active. */
bool jetson_uart_transport_send_async(jetson_uart_transport_t *transport,
                                      const uint8_t *data,
                                      size_t length);

#endif
