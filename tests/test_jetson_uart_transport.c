#include "jetson_uart_transport.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;

static HAL_StatusTypeDef receive_result = HAL_OK;
static HAL_StatusTypeDef stop_result = HAL_OK;
static unsigned receive_calls;
static unsigned stop_calls;

#define CHECK(condition)                                                                    \
    do                                                                                      \
    {                                                                                       \
        if (!(condition))                                                                   \
        {                                                                                   \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);               \
            return 1;                                                                       \
        }                                                                                   \
    } while (0)

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                               uint8_t *data,
                                               uint16_t size)
{
    CHECK(uart == &huart1);
    CHECK(data != NULL);
    CHECK(size == JETSON_UART_DMA_BUFFER_SIZE);
    ++receive_calls;
    return receive_result;
}

HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *uart)
{
    CHECK(uart == &huart1);
    ++stop_calls;
    return stop_result;
}

int main(void)
{
    static const uint8_t frame[] = {0xFF, 0xFC, 0x03, 0xB0, 0xB3};
    jetson_uart_transport_t transport;
    jetson_protocol_t parser;
    jetson_frame_t received;

    jetson_protocol_init(&parser);
    CHECK(jetson_uart_transport_start(&transport, &parser));
    CHECK(transport.state == JETSON_UART_RUNNING);
    CHECK(receive_calls == 1u);
    CHECK(!jetson_uart_transport_start(&transport, &parser));

    (void)memcpy(transport.rx_dma, frame, sizeof(frame));
    jetson_uart_transport_on_rx_event(&transport, sizeof(frame));
    jetson_uart_transport_on_rx_event(&transport, sizeof(frame));
    jetson_uart_transport_poll(&transport, 1u);
    CHECK(jetson_protocol_take_frame(&parser, &received));
    CHECK(received.function == JETSON_FUNC_INSPECT_REQUEST);
    CHECK(!jetson_protocol_take_frame(&parser, &received));

    CHECK(jetson_uart_transport_reset(&transport));
    transport.dma_position = JETSON_UART_DMA_BUFFER_SIZE - 2u;
    transport.rx_dma[JETSON_UART_DMA_BUFFER_SIZE - 2u] = frame[0];
    transport.rx_dma[JETSON_UART_DMA_BUFFER_SIZE - 1u] = frame[1];
    (void)memcpy(transport.rx_dma, &frame[2], sizeof(frame) - 2u);
    jetson_uart_transport_on_rx_event(&transport, 3u);
    jetson_uart_transport_poll(&transport, 2u);
    CHECK(jetson_protocol_take_frame(&parser, &received));

    CHECK(jetson_uart_transport_reset(&transport));
    transport.ring_head = JETSON_UART_SOFTWARE_RING_SIZE - 1u;
    transport.ring_tail = 0u;
    transport.rx_dma[0] = 0x55u;
    jetson_uart_transport_on_rx_event(&transport, 1u);
    CHECK(transport.stats.software_overruns == 1u);
    CHECK(transport.stats.dropped_bytes == 1u);

    CHECK(jetson_uart_transport_reset(&transport));
    jetson_protocol_feed(&parser, frame, 2u, 3u);
    CHECK(parser.used == 2u);
    jetson_uart_transport_on_uart_error(&transport);
    CHECK(transport.state == JETSON_UART_RECOVERY_PENDING);
    jetson_uart_transport_poll(&transport, 4u);
    CHECK(transport.state == JETSON_UART_RUNNING);
    CHECK(parser.used == 0u);
    CHECK(transport.stats.uart_errors == 1u);

    stop_result = HAL_ERROR;
    jetson_uart_transport_on_uart_error(&transport);
    jetson_uart_transport_poll(&transport, 5u);
    CHECK(transport.state == JETSON_UART_STOPPED);
    CHECK(transport.stats.dma_stop_failures == 1u);

    stop_result = HAL_OK;
    CHECK(jetson_uart_transport_reset(&transport));
    receive_result = HAL_ERROR;
    jetson_uart_transport_on_uart_error(&transport);
    jetson_uart_transport_poll(&transport, 6u);
    CHECK(transport.state == JETSON_UART_STOPPED);
    CHECK(transport.stats.restart_failures == 1u);
    CHECK(stop_calls == 7u);

    CHECK(!jetson_uart_transport_send_async(&transport, frame, sizeof(frame)));
    puts("jetson_uart_transport test passed");
    return 0;
}
