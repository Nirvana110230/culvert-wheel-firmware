#ifndef TEST_USART_H
#define TEST_USART_H

#include <stdint.h>

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR = 1
} HAL_StatusTypeDef;

typedef struct
{
    uint32_t unused;
} UART_HandleTypeDef;

extern UART_HandleTypeDef huart1;

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                               uint8_t *data,
                                               uint16_t size);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *uart);

#endif
