#ifndef USART2_RS485_ADAPTER_H
#define USART2_RS485_ADAPTER_H

#include "modbus_rx_assembler.h"
#include "mssd_driver.h"
#include "uart_dma_rx_buffer.h"

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define USART2_RS485_RX_DMA_SIZE 32u

typedef enum
{
    USART2_RS485_ADAPTER_UNINITIALIZED = 0,
    USART2_RS485_ADAPTER_STOPPED,
    USART2_RS485_ADAPTER_RUNNING,
    USART2_RS485_ADAPTER_RECOVERY_PENDING
} usart2_rs485_adapter_state_t;

typedef struct
{
    uint32_t rx_events;
    uint32_t invalid_rx_events;
    uint32_t rx_overruns;
    uint32_t accepted_frames;
    uint32_t invalid_frames;
    uint32_t unexpected_frames;
    uint32_t tx_dma_events;
    uint32_t invalid_tx_dma_events;
    uint32_t uart_tc_events;
    uint32_t uart_errors;
    uint32_t receive_start_failures;
    uint32_t receive_abort_failures;
    uint32_t transmit_start_failures;
    uint32_t transmit_abort_failures;
    uint32_t recovery_failures;
} usart2_rs485_adapter_stats_t;

typedef struct
{
    UART_HandleTypeDef *uart;
    DMA_HandleTypeDef *rx_dma;
    DMA_HandleTypeDef *tx_dma;
    GPIO_TypeDef *de_port;
    uint16_t de_pin;
    rs485_transport_t *transport;
    mssd_driver_t *driver;
    modbus_rx_assembler_t assembler;
    uart_dma_rx_buffer_t rx_buffer;
    uint8_t rx_dma_buffer[USART2_RS485_RX_DMA_SIZE];
    /* ISR sets these flags; poll() consumes them inside a short IRQ critical section. */
    volatile bool tx_dma_event_pending;
    volatile bool uart_error_pending;
    bool response_window_open;
    bool de_transmit;
    uint32_t poll_now_ms;
    usart2_rs485_adapter_state_t state;
    volatile usart2_rs485_adapter_stats_t stats;
} usart2_rs485_adapter_t;

/* Software-only initialization. It does not start DMA or change GPIO state. */
bool usart2_rs485_adapter_init(usart2_rs485_adapter_t *adapter,
                               UART_HandleTypeDef *uart,
                               DMA_HandleTypeDef *rx_dma,
                               DMA_HandleTypeDef *tx_dma,
                               GPIO_TypeDef *de_port,
                               uint16_t de_pin,
                               rs485_transport_t *transport,
                               mssd_driver_t *driver);

/* Returns the port callbacks used to initialize the existing RS485 transport. */
rs485_port_t usart2_rs485_adapter_get_port(usart2_rs485_adapter_t *adapter);

/* Hardware activation is explicit and is intentionally not called from main yet. */
bool usart2_rs485_adapter_start(usart2_rs485_adapter_t *adapter);
bool usart2_rs485_adapter_stop(usart2_rs485_adapter_t *adapter);
bool usart2_rs485_adapter_reset(usart2_rs485_adapter_t *adapter);
void usart2_rs485_adapter_poll(usart2_rs485_adapter_t *adapter, uint32_t now_ms);

/* Thin ISR dispatch entry points. They publish events only. */
void usart2_rs485_adapter_dispatch_rx_event(UART_HandleTypeDef *uart,
                                            uint16_t dma_position);
void usart2_rs485_adapter_dispatch_tx_dma_irq(DMA_HandleTypeDef *dma);
void usart2_rs485_adapter_dispatch_uart_tc(UART_HandleTypeDef *uart);
void usart2_rs485_adapter_dispatch_uart_error(UART_HandleTypeDef *uart);

#endif
