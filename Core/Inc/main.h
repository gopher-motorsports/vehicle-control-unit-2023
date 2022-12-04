/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define GPIO_1_Pin GPIO_PIN_1
#define GPIO_1_GPIO_Port GPIOC
#define GPIO_2_Pin GPIO_PIN_2
#define GPIO_2_GPIO_Port GPIOC
#define GPIO_3_Pin GPIO_PIN_3
#define GPIO_3_GPIO_Port GPIOC
#define STATUS_LED_Pin GPIO_PIN_0
#define STATUS_LED_GPIO_Port GPIOA
#define GSENSE_LED_Pin GPIO_PIN_1
#define GSENSE_LED_GPIO_Port GPIOA
#define HARDFAULT_LED_Pin GPIO_PIN_2
#define HARDFAULT_LED_GPIO_Port GPIOA
#define PUMP_PRES_Pin GPIO_PIN_5
#define PUMP_PRES_GPIO_Port GPIOA
#define APPS_1_Pin GPIO_PIN_6
#define APPS_1_GPIO_Port GPIOA
#define APPS_2_Pin GPIO_PIN_7
#define APPS_2_GPIO_Port GPIOA
#define TS_SNS_1_Pin GPIO_PIN_4
#define TS_SNS_1_GPIO_Port GPIOC
#define BRK_PRES_Pin GPIO_PIN_0
#define BRK_PRES_GPIO_Port GPIOB
#define RTD_BUZZER_Pin GPIO_PIN_1
#define RTD_BUZZER_GPIO_Port GPIOB
#define LED_RGB_R_Pin GPIO_PIN_12
#define LED_RGB_R_GPIO_Port GPIOB
#define LED_RGB_G_Pin GPIO_PIN_13
#define LED_RGB_G_GPIO_Port GPIOB
#define LED_RGB_B_Pin GPIO_PIN_14
#define LED_RGB_B_GPIO_Port GPIOB
#define PUMP_Pin GPIO_PIN_6
#define PUMP_GPIO_Port GPIOC
#define FAN_Pin GPIO_PIN_7
#define FAN_GPIO_Port GPIOC
#define BRK_LT_Pin GPIO_PIN_8
#define BRK_LT_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_9
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_10
#define USART_RX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define TS_SNS_FAULT_Pin GPIO_PIN_15
#define TS_SNS_FAULT_GPIO_Port GPIOA
#define APPS1_FAULT_Pin GPIO_PIN_10
#define APPS1_FAULT_GPIO_Port GPIOC
#define APPS2_FAULT_Pin GPIO_PIN_11
#define APPS2_FAULT_GPIO_Port GPIOC
#define BRK_FAULT_Pin GPIO_PIN_12
#define BRK_FAULT_GPIO_Port GPIOC
#define TS_BRK_FAULT_Pin GPIO_PIN_2
#define TS_BRK_FAULT_GPIO_Port GPIOD
#define SW_1_Pin GPIO_PIN_4
#define SW_1_GPIO_Port GPIOB
#define CANRX_Pin GPIO_PIN_5
#define CANRX_GPIO_Port GPIOB
#define CANTX_Pin GPIO_PIN_6
#define CANTX_GPIO_Port GPIOB
#define SW_2_Pin GPIO_PIN_7
#define SW_2_GPIO_Port GPIOB
#define SW_3_Pin GPIO_PIN_8
#define SW_3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
