#ifndef RS485_TRANSPORT_H
#define RS485_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RS485_TRANSPORT_MAX_REQUEST 256u

typedef enum
{
    RS485_IDLE = 0,
    RS485_SWITCH_TO_TX,
    RS485_DMA_TX,
    RS485_WAIT_UART_TC,
    RS485_SWITCH_TO_RX,
    RS485_WAIT_RESPONSE,
    RS485_COMPLETE,
    RS485_TIMEOUT,
    RS485_ERROR
} rs485_state_t;

typedef struct
{
    bool (*set_de)(bool transmit, void *context);
    bool (*start_dma)(const uint8_t *data, size_t length, void *context);
    bool (*uart_tc)(void *context);
    void *context;
} rs485_port_t;

typedef struct
{
    uint32_t dma_ms;
    uint32_t uart_tc_ms;
    uint32_t response_ms;
} rs485_timeouts_t;

typedef struct
{
    uint32_t invalid_callbacks;
    uint32_t transaction_errors;
    uint32_t timeouts;
    uint32_t aborts;
    uint32_t de_restore_failures;
} rs485_error_counters_t;

typedef struct
{
    rs485_state_t state;
    uint32_t state_enter_ms;
    rs485_port_t port;
    rs485_timeouts_t timeouts;
    rs485_error_counters_t errors;
    uint8_t request[RS485_TRANSPORT_MAX_REQUEST];
    size_t request_length;
} rs485_transport_t;

/* Initializes the transaction state and requests the safe receive direction. */
void rs485_transport_init(rs485_transport_t *transport,
                          rs485_port_t port,
                          rs485_timeouts_t timeouts);

/* Starts one transaction; succeeds only while the state is IDLE. */
bool rs485_transport_begin(rs485_transport_t *transport,
                           const uint8_t *request,
                           size_t request_length,
                           uint32_t now_ms);

/* Advances non-blocking direction and timeout states. */
void rs485_transport_poll(rs485_transport_t *transport, uint32_t now_ms);

/* Bound the pre-transmit wait for an idle line using the response budget. */
void rs485_transport_wait_for_bus(rs485_transport_t *transport, uint32_t now_ms);

/* These callbacks are accepted only in their matching active state. */
bool rs485_transport_dma_complete(rs485_transport_t *transport, uint32_t now_ms);
bool rs485_transport_uart_error(rs485_transport_t *transport, uint32_t now_ms);
bool rs485_transport_response_complete(rs485_transport_t *transport,
                                       uint32_t now_ms);

/* Aborts an active transaction and enters ERROR after attempting receive mode. */
bool rs485_transport_abort(rs485_transport_t *transport, uint32_t now_ms);

/* Acknowledges COMPLETE, TIMEOUT, or ERROR and returns to IDLE if DE is low. */
bool rs485_transport_acknowledge(rs485_transport_t *transport, uint32_t now_ms);

#endif
