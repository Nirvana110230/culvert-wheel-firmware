#include "usart2_rs485_adapter.h"

#include <limits.h>
#include <string.h>

static usart2_rs485_adapter_t *volatile active_adapter;

static bool port_set_de(bool transmit, void *context)
{
    usart2_rs485_adapter_t *adapter = context;

    if ((adapter == NULL) || (adapter->de_port == NULL) || (adapter->de_pin == 0u))
    {
        return false;
    }
    HAL_GPIO_WritePin(adapter->de_port,
                      adapter->de_pin,
                      transmit ? GPIO_PIN_SET : GPIO_PIN_RESET);
    adapter->de_transmit = transmit;
    return true;
}

static bool port_start_dma(const uint8_t *data, size_t length, void *context)
{
    usart2_rs485_adapter_t *adapter = context;

    if ((adapter == NULL) || (adapter->state != USART2_RS485_ADAPTER_RUNNING) ||
        (data == NULL) || (length == 0u) || (length > UINT16_MAX) ||
        (HAL_UART_Transmit_DMA(adapter->uart,
                               data,
                               (uint16_t)length) != HAL_OK))
    {
        if (adapter != NULL)
        {
            ++adapter->stats.transmit_start_failures;
        }
        return false;
    }
    return true;
}

static bool port_uart_tc(void *context)
{
    const usart2_rs485_adapter_t *adapter = context;

    return (adapter != NULL) &&
           (__HAL_UART_GET_FLAG(adapter->uart, UART_FLAG_TC) != RESET);
}

static bool take_isr_flag(volatile bool *flag)
{
    const uint32_t primask = __get_PRIMASK();
    bool was_set;

    __disable_irq();
    was_set = *flag;
    *flag = false;
    if (primask == 0u)
    {
        __enable_irq();
    }
    return was_set;
}

static bool transport_is_active(const rs485_transport_t *transport)
{
    return (transport->state == RS485_SWITCH_TO_TX) ||
           (transport->state == RS485_DMA_TX) ||
           (transport->state == RS485_WAIT_UART_TC) ||
           (transport->state == RS485_SWITCH_TO_RX) ||
           (transport->state == RS485_WAIT_RESPONSE);
}

static void clear_receive_pipeline(usart2_rs485_adapter_t *adapter)
{
    (void)take_isr_flag(&adapter->tx_dma_event_pending);
    (void)take_isr_flag(&adapter->uart_error_pending);
    modbus_rx_assembler_reset(&adapter->assembler);
    uart_dma_rx_buffer_reset_stream(&adapter->rx_buffer);
    adapter->response_window_open = false;
}

static bool start_receive(usart2_rs485_adapter_t *adapter)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(adapter->uart,
                                     adapter->rx_dma_buffer,
                                     (uint16_t)sizeof(adapter->rx_dma_buffer)) != HAL_OK)
    {
        ++adapter->stats.receive_start_failures;
        return false;
    }
    __HAL_DMA_DISABLE_IT(adapter->rx_dma, DMA_IT_HT);
    return true;
}

static bool restart_receive(usart2_rs485_adapter_t *adapter, bool abort_transmit)
{
    bool success = true;

    if (abort_transmit && (HAL_UART_AbortTransmit(adapter->uart) != HAL_OK))
    {
        ++adapter->stats.transmit_abort_failures;
        success = false;
    }
    if (HAL_UART_AbortReceive(adapter->uart) != HAL_OK)
    {
        ++adapter->stats.receive_abort_failures;
        success = false;
    }
    clear_receive_pipeline(adapter);
    if (!port_set_de(false, adapter) || !start_receive(adapter))
    {
        success = false;
    }
    adapter->state = success ? USART2_RS485_ADAPTER_RUNNING :
                               USART2_RS485_ADAPTER_STOPPED;
    if (!success)
    {
        ++adapter->stats.recovery_failures;
    }
    return success;
}

static bool deliver_candidate(const uint8_t *frame, size_t length, void *context)
{
    usart2_rs485_adapter_t *adapter = context;
    const mssd_response_status_t status =
        mssd_driver_on_response(adapter->driver,
                                frame,
                                length,
                                adapter->poll_now_ms);

    if (status == MSSD_RESPONSE_INVALID_FRAME)
    {
        ++adapter->stats.invalid_frames;
        if (mssd_driver_get_error(adapter->driver).modbus_status ==
            MODBUS_RTU_VALIDATION_CRC_ERROR)
        {
            ++adapter->stats.crc_errors;
        }
        return true;
    }
    if (status == MSSD_RESPONSE_ACCEPTED)
    {
        ++adapter->stats.accepted_frames;
    }
    else
    {
        ++adapter->stats.unexpected_frames;
    }
    adapter->response_window_open = false;
    return false;
}

bool usart2_rs485_adapter_init(usart2_rs485_adapter_t *adapter,
                               UART_HandleTypeDef *uart,
                               DMA_HandleTypeDef *rx_dma,
                               DMA_HandleTypeDef *tx_dma,
                               GPIO_TypeDef *de_port,
                               uint16_t de_pin,
                               rs485_transport_t *transport,
                               mssd_driver_t *driver)
{
    if ((adapter == NULL) || (uart == NULL) || (rx_dma == NULL) ||
        (tx_dma == NULL) || (de_port == NULL) || (de_pin == 0u) ||
        (transport == NULL) || (driver == NULL))
    {
        return false;
    }

    (void)memset(adapter, 0, sizeof(*adapter));
    adapter->uart = uart;
    adapter->rx_dma = rx_dma;
    adapter->tx_dma = tx_dma;
    adapter->de_port = de_port;
    adapter->de_pin = de_pin;
    adapter->transport = transport;
    adapter->driver = driver;
    adapter->state = USART2_RS485_ADAPTER_STOPPED;
    modbus_rx_assembler_init(&adapter->assembler);
    uart_dma_rx_buffer_init(&adapter->rx_buffer);
    return true;
}

rs485_port_t usart2_rs485_adapter_get_port(usart2_rs485_adapter_t *adapter)
{
    rs485_port_t port = {0};

    if (adapter != NULL)
    {
        port.set_de = port_set_de;
        port.start_dma = port_start_dma;
        port.uart_tc = port_uart_tc;
        port.context = adapter;
    }
    return port;
}

bool usart2_rs485_adapter_start(usart2_rs485_adapter_t *adapter)
{
    if ((adapter == NULL) || (adapter->state != USART2_RS485_ADAPTER_STOPPED) ||
        (active_adapter != NULL) ||
        (adapter->transport->port.context != adapter) ||
        (adapter->driver->transport != adapter->transport))
    {
        return false;
    }

    clear_receive_pipeline(adapter);
    if (!port_set_de(false, adapter))
    {
        return false;
    }
    active_adapter = adapter;
    if (!start_receive(adapter))
    {
        active_adapter = NULL;
        return false;
    }
    adapter->state = USART2_RS485_ADAPTER_RUNNING;
    return true;
}

bool usart2_rs485_adapter_stop(usart2_rs485_adapter_t *adapter)
{
    bool success = true;

    if ((adapter == NULL) || (active_adapter != adapter))
    {
        return false;
    }
    if (HAL_UART_AbortTransmit(adapter->uart) != HAL_OK)
    {
        ++adapter->stats.transmit_abort_failures;
        success = false;
    }
    if (HAL_UART_AbortReceive(adapter->uart) != HAL_OK)
    {
        ++adapter->stats.receive_abort_failures;
        success = false;
    }
    if (!port_set_de(false, adapter))
    {
        success = false;
    }
    clear_receive_pipeline(adapter);
    adapter->state = USART2_RS485_ADAPTER_STOPPED;
    active_adapter = NULL;
    return success;
}

bool usart2_rs485_adapter_reset(usart2_rs485_adapter_t *adapter)
{
    if ((adapter == NULL) || (active_adapter != adapter) ||
        ((adapter->transport->state != RS485_IDLE) &&
         (adapter->transport->state != RS485_COMPLETE) &&
         (adapter->transport->state != RS485_TIMEOUT) &&
         (adapter->transport->state != RS485_ERROR)))
    {
        return false;
    }
    adapter->state = USART2_RS485_ADAPTER_RECOVERY_PENDING;
    return restart_receive(adapter, false);
}

void usart2_rs485_adapter_poll(usart2_rs485_adapter_t *adapter, uint32_t now_ms)
{
    bool pipeline_error = false;
    uint8_t byte;

    if ((adapter == NULL) || (active_adapter != adapter) ||
        (adapter->state == USART2_RS485_ADAPTER_STOPPED))
    {
        return;
    }
    adapter->poll_now_ms = now_ms;

    if (take_isr_flag(&adapter->uart_error_pending))
    {
        ++adapter->stats.uart_errors;
        pipeline_error = true;
    }
    if (uart_dma_rx_buffer_take_overflow(&adapter->rx_buffer))
    {
        ++adapter->stats.rx_overruns;
        pipeline_error = true;
    }
    if (pipeline_error)
    {
        if (transport_is_active(adapter->transport))
        {
            (void)rs485_transport_uart_error(adapter->transport, now_ms);
            mssd_driver_poll(adapter->driver, now_ms);
        }
        adapter->state = USART2_RS485_ADAPTER_RECOVERY_PENDING;
        (void)restart_receive(adapter, true);
        return;
    }

    if (take_isr_flag(&adapter->tx_dma_event_pending))
    {
        ++adapter->stats.tx_dma_events;
        if (!rs485_transport_dma_complete(adapter->transport, now_ms))
        {
            ++adapter->stats.invalid_tx_dma_events;
        }
    }

    mssd_driver_poll(adapter->driver, now_ms);
    if (adapter->transport->state != RS485_WAIT_RESPONSE)
    {
        if (adapter->response_window_open ||
            (modbus_rx_assembler_pending_length(&adapter->assembler) != 0u))
        {
            modbus_rx_assembler_reset(&adapter->assembler);
        }
        adapter->response_window_open = false;
        uart_dma_rx_buffer_discard(&adapter->rx_buffer);
        return;
    }

    if (!adapter->response_window_open)
    {
        modbus_rx_assembler_reset(&adapter->assembler);
        adapter->response_window_open = true;
    }
    while (adapter->response_window_open &&
           uart_dma_rx_buffer_take(&adapter->rx_buffer, &byte))
    {
        (void)modbus_rx_assembler_feed(&adapter->assembler,
                                       &byte,
                                       1u,
                                       deliver_candidate,
                                       adapter);
    }
    if (!adapter->response_window_open)
    {
        modbus_rx_assembler_reset(&adapter->assembler);
        uart_dma_rx_buffer_discard(&adapter->rx_buffer);
    }
}

void usart2_rs485_adapter_dispatch_rx_event(UART_HandleTypeDef *uart,
                                            uint16_t dma_position)
{
    usart2_rs485_adapter_t *adapter = active_adapter;

    if ((adapter == NULL) || (uart != adapter->uart) ||
        (adapter->state != USART2_RS485_ADAPTER_RUNNING))
    {
        return;
    }
    ++adapter->stats.rx_events;
    if (!uart_dma_rx_buffer_publish(&adapter->rx_buffer,
                                    adapter->rx_dma_buffer,
                                    sizeof(adapter->rx_dma_buffer),
                                    dma_position,
                                    HAL_UARTEx_GetRxEventType(uart) ==
                                        HAL_UART_RXEVENT_TC))
    {
        ++adapter->stats.invalid_rx_events;
    }
}

void usart2_rs485_adapter_dispatch_tx_dma_irq(DMA_HandleTypeDef *dma)
{
    usart2_rs485_adapter_t *adapter = active_adapter;

    if ((adapter == NULL) || (dma != adapter->tx_dma))
    {
        return;
    }
    if (HAL_DMA_GetError(dma) == HAL_DMA_ERROR_NONE)
    {
        adapter->tx_dma_event_pending = true;
    }
    else
    {
        ++adapter->stats.dma_errors;
        adapter->uart_error_pending = true;
    }
}

void usart2_rs485_adapter_dispatch_uart_tc(UART_HandleTypeDef *uart)
{
    usart2_rs485_adapter_t *adapter = active_adapter;

    if ((adapter != NULL) && (uart == adapter->uart))
    {
        ++adapter->stats.uart_tc_events;
    }
}

void usart2_rs485_adapter_dispatch_uart_error(UART_HandleTypeDef *uart)
{
    usart2_rs485_adapter_t *adapter = active_adapter;

    if ((adapter != NULL) && (uart == adapter->uart))
    {
        adapter->uart_error_pending = true;
    }
}
