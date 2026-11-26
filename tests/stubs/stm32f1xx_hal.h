#ifndef TEST_STM32F1XX_HAL_H
#define TEST_STM32F1XX_HAL_H

#include <stdint.h>

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2,
    HAL_TIMEOUT = 3
} HAL_StatusTypeDef;

typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

typedef struct
{
    uint32_t error;
    uint32_t state;
    uint32_t remaining;
} DMA_HandleTypeDef;

#define HAL_DMA_STATE_READY 0u
#define HAL_DMA_STATE_BUSY 1u
#define HAL_DMA_GetState(dma) ((dma)->state)
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)

typedef struct
{
    uint32_t writes;
    GPIO_PinState last_state;
} GPIO_TypeDef;

typedef struct __UART_HandleTypeDef
{
    void *Instance;
    DMA_HandleTypeDef *hdmarx;
    DMA_HandleTypeDef *hdmatx;
    uint32_t rx_event_type;
    uint32_t tc_flag;
} UART_HandleTypeDef;

#define HAL_UART_RXEVENT_TC 0u
#define HAL_UART_RXEVENT_HT 1u
#define HAL_UART_RXEVENT_IDLE 2u
#define HAL_DMA_ERROR_NONE 0u
#define DMA_IT_HT 0x04u
#define UART_FLAG_TC 0x40u
#define RESET 0u

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                               uint8_t *data,
                                               uint16_t size);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart,
                                        const uint8_t *data,
                                        uint16_t size);
uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *uart);
uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *dma);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
void test_hal_dma_disable_it(DMA_HandleTypeDef *dma, uint32_t interrupt);
uint32_t test_hal_uart_get_flag(UART_HandleTypeDef *uart, uint32_t flag);
uint32_t __get_PRIMASK(void);
uint32_t HAL_GetTick(void);
void __disable_irq(void);
void __enable_irq(void);

#define __HAL_DMA_DISABLE_IT(dma, interrupt) test_hal_dma_disable_it((dma), (interrupt))
#define __HAL_UART_GET_FLAG(uart, flag) test_hal_uart_get_flag((uart), (flag))

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart);

#endif
