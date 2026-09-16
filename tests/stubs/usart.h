#ifndef TEST_USART_H
#define TEST_USART_H

#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                               uint8_t *data,
                                               uint16_t size);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *uart);

#endif
