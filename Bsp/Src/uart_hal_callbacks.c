#include "jetson_uart_transport.h"
#include "usart2_rs485_adapter.h"

#include "usart.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if (uart == &huart1)
    {
        jetson_uart_transport_dispatch_rx_event(size, HAL_GetTick());
    }
    else
    {
        usart2_rs485_adapter_dispatch_rx_event(uart, size, HAL_GetTick());
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1)
    {
        jetson_uart_transport_dispatch_tx_complete();
    }
    else
    {
        usart2_rs485_adapter_dispatch_uart_tc(uart);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1)
    {
        jetson_uart_transport_dispatch_uart_error();
    }
    else
    {
        usart2_rs485_adapter_dispatch_uart_error(uart);
    }
}
