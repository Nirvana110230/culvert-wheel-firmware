/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOE
#define MSSD_TX_Pin GPIO_PIN_2
#define MSSD_TX_GPIO_Port GPIOA
#define MSSD_RX_Pin GPIO_PIN_3
#define MSSD_RX_GPIO_Port GPIOA
#define RC_CH7_ESTOP_Pin GPIO_PIN_6
#define RC_CH7_ESTOP_GPIO_Port GPIOA
#define RC_CH1_Pin GPIO_PIN_6
#define RC_CH1_GPIO_Port GPIOC
#define RC_CH2_Pin GPIO_PIN_7
#define RC_CH2_GPIO_Port GPIOC
#define RC_CH5_Pin GPIO_PIN_8
#define RC_CH5_GPIO_Port GPIOC
#define RC_CH6_Pin GPIO_PIN_9
#define RC_CH6_GPIO_Port GPIOC
#define JETSON_TX_Pin GPIO_PIN_9
#define JETSON_TX_GPIO_Port GPIOA
#define JETSON_RX_Pin GPIO_PIN_10
#define JETSON_RX_GPIO_Port GPIOA
#define RS485_DE_Pin GPIO_PIN_7
#define RS485_DE_GPIO_Port GPIOD
#define LED0_Pin GPIO_PIN_5
#define LED0_GPIO_Port GPIOB
#define BEEP_Pin GPIO_PIN_8
#define BEEP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
