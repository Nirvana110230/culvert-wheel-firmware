#include "usart2_rs485_adapter.h"
#include "usart.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

static HAL_StatusTypeDef receive_result = HAL_OK;
static HAL_StatusTypeDef abort_result = HAL_OK;
static HAL_StatusTypeDef transmit_result = HAL_OK;
static unsigned receive_calls;
static unsigned abort_calls;
static unsigned transmit_abort_calls;
static unsigned transmit_calls;
static unsigned disable_ht_calls;
static unsigned jetson_rx_events;
static unsigned jetson_errors;
static uint32_t interrupt_mask;
static bool hal_contract_failed;

#define CHECK(condition)                                                               \
    do                                                                                 \
    {                                                                                  \
        if (!(condition))                                                              \
        {                                                                              \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);         \
            return false;                                                              \
        }                                                                              \
    } while (0)

#define RUN(test)                                                                      \
    do                                                                                 \
    {                                                                                  \
        if (!(test)())                                                                 \
        {                                                                              \
            return 1;                                                                  \
        }                                                                              \
    } while (0)

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                               uint8_t *data,
                                               uint16_t size)
{
    if ((uart != &huart2) || (data == NULL) ||
        (size != USART2_RS485_RX_DMA_SIZE))
    {
        hal_contract_failed = true;
        return HAL_ERROR;
    }
    ++receive_calls;
    return receive_result;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    if (uart != &huart2)
    {
        hal_contract_failed = true;
        return HAL_ERROR;
    }
    ++abort_calls;
    return abort_result;
}

HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart)
{
    if (uart != &huart2)
    {
        hal_contract_failed = true;
        return HAL_ERROR;
    }
    ++transmit_abort_calls;
    return abort_result;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart,
                                        const uint8_t *data,
                                        uint16_t size)
{
    if ((uart != &huart2) || (data == NULL) || (size == 0u))
    {
        hal_contract_failed = true;
        return HAL_ERROR;
    }
    ++transmit_calls;
    return transmit_result;
}

uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *uart)
{
    return uart->rx_event_type;
}

uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *dma)
{
    return dma->error;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if ((port == NULL) || (pin != 0x0080u))
    {
        hal_contract_failed = true;
        return;
    }
    ++port->writes;
    port->last_state = state;
}

void test_hal_dma_disable_it(DMA_HandleTypeDef *dma, uint32_t interrupt)
{
    if ((dma == NULL) || (interrupt != DMA_IT_HT))
    {
        hal_contract_failed = true;
        return;
    }
    ++disable_ht_calls;
}

uint32_t test_hal_uart_get_flag(UART_HandleTypeDef *uart, uint32_t flag)
{
    if ((uart != &huart2) || (flag != UART_FLAG_TC))
    {
        hal_contract_failed = true;
        return 0u;
    }
    return uart->tc_flag;
}

uint32_t __get_PRIMASK(void)
{
    return interrupt_mask;
}

void __disable_irq(void)
{
    interrupt_mask = 1u;
}

void __enable_irq(void)
{
    interrupt_mask = 0u;
}

void jetson_uart_transport_dispatch_rx_event(uint16_t size)
{
    (void)size;
    ++jetson_rx_events;
}

void jetson_uart_transport_dispatch_uart_error(void)
{
    ++jetson_errors;
}

static uint16_t crc16(const uint8_t *data, size_t length)
{
    return modbus_rtu_crc16(data, length);
}

static void append_crc(uint8_t *frame, size_t payload_length)
{
    const uint16_t crc = crc16(frame, payload_length);

    frame[payload_length] = (uint8_t)crc;
    frame[payload_length + 1u] = (uint8_t)(crc >> 8u);
}

static void reset_hal(void)
{
    receive_result = HAL_OK;
    abort_result = HAL_OK;
    transmit_result = HAL_OK;
    receive_calls = 0u;
    abort_calls = 0u;
    transmit_abort_calls = 0u;
    transmit_calls = 0u;
    disable_ht_calls = 0u;
    jetson_rx_events = 0u;
    jetson_errors = 0u;
    interrupt_mask = 0u;
    hal_contract_failed = false;
    (void)memset(&huart1, 0, sizeof(huart1));
    (void)memset(&huart2, 0, sizeof(huart2));
}

static bool setup(usart2_rs485_adapter_t *adapter,
                  rs485_transport_t *transport,
                  mssd_driver_t *driver,
                  DMA_HandleTypeDef *rx_dma,
                  DMA_HandleTypeDef *tx_dma,
                  GPIO_TypeDef *de_port)
{
    const rs485_timeouts_t timeouts = {10u, 10u, 20u};
    const mssd_driver_config_t config = mssd_driver_default_config();
    rs485_port_t port;

    reset_hal();
    huart2.hdmarx = rx_dma;
    huart2.hdmatx = tx_dma;
    CHECK(usart2_rs485_adapter_init(adapter,
                                    &huart2,
                                    rx_dma,
                                    tx_dma,
                                    de_port,
                                    0x0080u,
                                    transport,
                                    driver));
    port = usart2_rs485_adapter_get_port(adapter);
    rs485_transport_init(transport, port, timeouts);
    CHECK(mssd_driver_init(driver, transport, &config));
    CHECK(usart2_rs485_adapter_start(adapter));
    CHECK(receive_calls == 1u);
    CHECK(disable_ht_calls == 1u);
    CHECK(de_port->last_state == GPIO_PIN_RESET);
    CHECK(!hal_contract_failed);
    return true;
}

static bool reach_response_wait(usart2_rs485_adapter_t *adapter,
                                 rs485_transport_t *transport,
                                 DMA_HandleTypeDef *tx_dma,
                                 uint32_t start_ms)
{
    const uint32_t uart_tc_events_before = adapter->stats.uart_tc_events;

    huart2.tc_flag = 0u;
    usart2_rs485_adapter_poll(adapter, start_ms);
    CHECK(transport->state == RS485_DMA_TX);
    usart2_rs485_adapter_dispatch_tx_dma_irq(tx_dma);
    CHECK(transport->state == RS485_DMA_TX);
    usart2_rs485_adapter_poll(adapter, start_ms + 1u);
    CHECK(transport->state == RS485_WAIT_UART_TC);
    huart2.tc_flag = UART_FLAG_TC;
    HAL_UART_TxCpltCallback(&huart2);
    usart2_rs485_adapter_poll(adapter, start_ms + 2u);
    CHECK(transport->state == RS485_SWITCH_TO_RX);
    usart2_rs485_adapter_poll(adapter, start_ms + 3u);
    CHECK(transport->state == RS485_WAIT_RESPONSE);
    CHECK(adapter->stats.uart_tc_events == uart_tc_events_before + 1u);
    return true;
}

static bool test_rx_frame_reaches_mssd_only_in_foreground(void)
{
    uint8_t response[] = {
        0x01u, 0x03u, 0x08u,
        0x00u, 0x01u,
        0x00u, 0x00u, 0x03u, 0xE8u,
        0x00u, 0x00u,
        0x00u, 0x00u,
    };
    usart2_rs485_adapter_t adapter;
    rs485_transport_t transport;
    mssd_driver_t driver;
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};
    mssd_motor_status_t status;

    CHECK(setup(&adapter, &transport, &driver, &rx_dma, &tx_dma, &de_port));
    CHECK(mssd_driver_request_status(&driver, MSSD_CHANNEL_RIGHT, 0u) ==
          MSSD_REQUEST_ACCEPTED);
    CHECK(reach_response_wait(&adapter, &transport, &tx_dma, 0u));
    append_crc(response, sizeof(response) - 2u);
    (void)memcpy(adapter.rx_dma_buffer, response, sizeof(response));
    huart2.rx_event_type = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart2, (uint16_t)sizeof(response));
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_WAITING_RESPONSE);
    usart2_rs485_adapter_poll(&adapter, 4u);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_COMPLETE);
    CHECK(mssd_driver_get_status(&driver, MSSD_CHANNEL_RIGHT, &status));
    CHECK(status.rpm == 1000);
    CHECK(usart2_rs485_adapter_stop(&adapter));
    return true;
}

static bool test_invalid_crc_then_valid_response(void)
{
    uint8_t invalid[] = {
        0x01u, 0x06u, 0x00u, 0x38u, 0x00u, 0x00u, 0x00u, 0x00u,
    };
    uint8_t valid[sizeof(invalid)];
    usart2_rs485_adapter_t adapter;
    rs485_transport_t transport;
    mssd_driver_t driver;
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};

    CHECK(setup(&adapter, &transport, &driver, &rx_dma, &tx_dma, &de_port));
    CHECK(mssd_driver_request_stop(&driver,
                                   MSSD_CHANNEL_RIGHT,
                                   MSSD_STOP_NORMAL,
                                   0u) == MSSD_REQUEST_ACCEPTED);
    CHECK(reach_response_wait(&adapter, &transport, &tx_dma, 0u));
    (void)memcpy(valid, transport.request, sizeof(valid));
    (void)memcpy(adapter.rx_dma_buffer, invalid, sizeof(invalid));
    (void)memcpy(&adapter.rx_dma_buffer[sizeof(invalid)], valid, sizeof(valid));
    huart2.rx_event_type = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart2, (uint16_t)(sizeof(invalid) + sizeof(valid)));
    usart2_rs485_adapter_poll(&adapter, 4u);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_COMPLETE);
    CHECK(adapter.stats.invalid_frames == 1u);
    CHECK(adapter.stats.accepted_frames == 1u);
    CHECK(usart2_rs485_adapter_stop(&adapter));
    return true;
}

static bool test_overflow_forms_error_and_recovers_rx(void)
{
    usart2_rs485_adapter_t adapter;
    rs485_transport_t transport;
    mssd_driver_t driver;
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};
    size_t index;

    CHECK(setup(&adapter, &transport, &driver, &rx_dma, &tx_dma, &de_port));
    CHECK(mssd_driver_request_status(&driver, MSSD_CHANNEL_RIGHT, 0u) ==
          MSSD_REQUEST_ACCEPTED);
    CHECK(reach_response_wait(&adapter, &transport, &tx_dma, 0u));
    for (index = 0u; index < sizeof(adapter.rx_dma_buffer); ++index)
    {
        adapter.rx_dma_buffer[index] = 0x55u;
    }
    huart2.rx_event_type = HAL_UART_RXEVENT_TC;
    HAL_UARTEx_RxEventCallback(&huart2, USART2_RS485_RX_DMA_SIZE);
    HAL_UARTEx_RxEventCallback(&huart2, USART2_RS485_RX_DMA_SIZE);
    CHECK(adapter.rx_buffer.overflow_pending);
    usart2_rs485_adapter_poll(&adapter, 4u);
    CHECK(adapter.stats.rx_overruns == 1u);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_FAILED);
    CHECK(abort_calls == 1u);
    CHECK(receive_calls == 2u);
    CHECK(de_port.last_state == GPIO_PIN_RESET);
    CHECK(usart2_rs485_adapter_stop(&adapter));
    return true;
}

static bool test_uart_error_is_deferred_and_recovers(void)
{
    usart2_rs485_adapter_t adapter;
    rs485_transport_t transport;
    mssd_driver_t driver;
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};

    CHECK(setup(&adapter, &transport, &driver, &rx_dma, &tx_dma, &de_port));
    CHECK(mssd_driver_request_stop(&driver,
                                   MSSD_CHANNEL_RIGHT,
                                   MSSD_STOP_NORMAL,
                                   0u) == MSSD_REQUEST_ACCEPTED);
    usart2_rs485_adapter_poll(&adapter, 0u);
    CHECK(transport.state == RS485_DMA_TX);
    HAL_UART_ErrorCallback(&huart2);
    CHECK(transport.state == RS485_DMA_TX);
    usart2_rs485_adapter_poll(&adapter, 1u);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_FAILED);
    CHECK(adapter.stats.uart_errors == 1u);
    CHECK(de_port.last_state == GPIO_PIN_RESET);
    CHECK(usart2_rs485_adapter_stop(&adapter));
    return true;
}

static bool test_usart1_callbacks_are_isolated(void)
{
    usart2_rs485_adapter_t adapter;
    rs485_transport_t transport;
    mssd_driver_t driver;
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};

    CHECK(setup(&adapter, &transport, &driver, &rx_dma, &tx_dma, &de_port));
    HAL_UARTEx_RxEventCallback(&huart1, 7u);
    HAL_UART_ErrorCallback(&huart1);
    HAL_UART_TxCpltCallback(&huart1);
    CHECK(jetson_rx_events == 1u);
    CHECK(jetson_errors == 1u);
    CHECK(adapter.stats.rx_events == 0u);
    CHECK(adapter.stats.uart_errors == 0u);
    CHECK(adapter.stats.uart_tc_events == 0u);
    CHECK(usart2_rs485_adapter_stop(&adapter));
    return true;
}

static bool test_explicit_reset_clears_partial_data(void)
{
    usart2_rs485_adapter_t adapter;
    rs485_transport_t transport;
    mssd_driver_t driver;
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};

    CHECK(setup(&adapter, &transport, &driver, &rx_dma, &tx_dma, &de_port));
    adapter.assembler.buffer[0] = 1u;
    adapter.assembler.used = 1u;
    adapter.rx_buffer.ring[0] = 2u;
    adapter.rx_buffer.head = 1u;
    CHECK(usart2_rs485_adapter_reset(&adapter));
    CHECK(modbus_rx_assembler_pending_length(&adapter.assembler) == 0u);
    CHECK(uart_dma_rx_buffer_count(&adapter.rx_buffer) == 0u);
    CHECK(abort_calls == 1u);
    CHECK(receive_calls == 2u);
    CHECK(usart2_rs485_adapter_stop(&adapter));
    return true;
}

static bool test_transaction_end_discards_residual_bytes(void)
{
    uint8_t response[8];
    usart2_rs485_adapter_t adapter;
    rs485_transport_t transport;
    mssd_driver_t driver;
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};

    CHECK(setup(&adapter, &transport, &driver, &rx_dma, &tx_dma, &de_port));
    CHECK(mssd_driver_request_stop(&driver,
                                   MSSD_CHANNEL_RIGHT,
                                   MSSD_STOP_NORMAL,
                                   0u) == MSSD_REQUEST_ACCEPTED);
    CHECK(reach_response_wait(&adapter, &transport, &tx_dma, 0u));
    (void)memcpy(response, transport.request, sizeof(response));
    (void)memcpy(adapter.rx_dma_buffer, response, sizeof(response));
    adapter.rx_dma_buffer[8] = 0x01u;
    adapter.rx_dma_buffer[9] = 0x03u;
    adapter.rx_dma_buffer[10] = 0x08u;
    huart2.rx_event_type = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart2, 11u);
    usart2_rs485_adapter_poll(&adapter, 4u);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_COMPLETE);
    CHECK(modbus_rx_assembler_pending_length(&adapter.assembler) == 0u);
    CHECK(uart_dma_rx_buffer_count(&adapter.rx_buffer) == 0u);

    CHECK(mssd_driver_acknowledge(&driver, 5u));
    CHECK(mssd_driver_request_stop(&driver,
                                   MSSD_CHANNEL_LEFT,
                                   MSSD_STOP_NORMAL,
                                   6u) == MSSD_REQUEST_ACCEPTED);
    CHECK(reach_response_wait(&adapter, &transport, &tx_dma, 6u));
    (void)memcpy(response, transport.request, sizeof(response));
    (void)memcpy(&adapter.rx_dma_buffer[11], response, sizeof(response));
    huart2.rx_event_type = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart2, 19u);
    usart2_rs485_adapter_poll(&adapter, 10u);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_COMPLETE);
    CHECK(usart2_rs485_adapter_stop(&adapter));
    return true;
}

static bool test_timeout_clears_partial_frame(void)
{
    usart2_rs485_adapter_t adapter;
    rs485_transport_t transport;
    mssd_driver_t driver;
    DMA_HandleTypeDef rx_dma = {0};
    DMA_HandleTypeDef tx_dma = {0};
    GPIO_TypeDef de_port = {0};

    CHECK(setup(&adapter, &transport, &driver, &rx_dma, &tx_dma, &de_port));
    CHECK(mssd_driver_request_status(&driver, MSSD_CHANNEL_RIGHT, 0u) ==
          MSSD_REQUEST_ACCEPTED);
    CHECK(reach_response_wait(&adapter, &transport, &tx_dma, 0u));
    adapter.rx_dma_buffer[0] = 0x01u;
    adapter.rx_dma_buffer[1] = 0x03u;
    adapter.rx_dma_buffer[2] = 0x08u;
    huart2.rx_event_type = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart2, 3u);
    usart2_rs485_adapter_poll(&adapter, 4u);
    CHECK(modbus_rx_assembler_pending_length(&adapter.assembler) == 3u);
    usart2_rs485_adapter_poll(&adapter, 23u);
    CHECK(mssd_driver_get_state(&driver) == MSSD_DRIVER_FAILED);
    CHECK(modbus_rx_assembler_pending_length(&adapter.assembler) == 0u);
    CHECK(uart_dma_rx_buffer_count(&adapter.rx_buffer) == 0u);
    CHECK(usart2_rs485_adapter_stop(&adapter));
    return true;
}

int main(void)
{
    RUN(test_rx_frame_reaches_mssd_only_in_foreground);
    RUN(test_invalid_crc_then_valid_response);
    RUN(test_overflow_forms_error_and_recovers_rx);
    RUN(test_uart_error_is_deferred_and_recovers);
    RUN(test_usart1_callbacks_are_isolated);
    RUN(test_explicit_reset_clears_partial_data);
    RUN(test_transaction_end_discards_residual_bytes);
    RUN(test_timeout_clears_partial_frame);
    puts("all 8 usart2_rs485_adapter tests passed");
    return 0;
}
