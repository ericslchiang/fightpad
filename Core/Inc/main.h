/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOA
#define BTN_L2_Pin GPIO_PIN_1
#define BTN_L2_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define BTN_UP_Pin GPIO_PIN_3
#define BTN_UP_GPIO_Port GPIOA
#define BTN_DOWN_Pin GPIO_PIN_4
#define BTN_DOWN_GPIO_Port GPIOA
#define BTN_RIGHT_Pin GPIO_PIN_5
#define BTN_RIGHT_GPIO_Port GPIOA
#define BTN_LEFT_Pin GPIO_PIN_6
#define BTN_LEFT_GPIO_Port GPIOA
#define BTN_B1_Pin GPIO_PIN_7
#define BTN_B1_GPIO_Port GPIOA
#define BTN_B2_Pin GPIO_PIN_0
#define BTN_B2_GPIO_Port GPIOB
#define BTN_B3_Pin GPIO_PIN_1
#define BTN_B3_GPIO_Port GPIOB
#define BTN_B4_Pin GPIO_PIN_8
#define BTN_B4_GPIO_Port GPIOA
#define BTN_R1_Pin GPIO_PIN_9
#define BTN_R1_GPIO_Port GPIOA
#define BTN_L1_Pin GPIO_PIN_10
#define BTN_L1_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define BTN_R2_Pin GPIO_PIN_3
#define BTN_R2_GPIO_Port GPIOB
#define BTN_S2_Pin GPIO_PIN_4
#define BTN_S2_GPIO_Port GPIOB
#define BTN_S1_Pin GPIO_PIN_5
#define BTN_S1_GPIO_Port GPIOB
#define BTN_R3_Pin GPIO_PIN_6
#define BTN_R3_GPIO_Port GPIOB
#define BTN_L3_Pin GPIO_PIN_7
#define BTN_L3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
