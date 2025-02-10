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
#include "stm32f0xx_hal.h"

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
#define RIGHT_Pin GPIO_PIN_0
#define RIGHT_GPIO_Port GPIOA
#define SELECT_Pin GPIO_PIN_1
#define SELECT_GPIO_Port GPIOA
#define LEFT_Pin GPIO_PIN_2
#define LEFT_GPIO_Port GPIOA
#define FV_TEST_Pin GPIO_PIN_3
#define FV_TEST_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOA
#define M2_Pin GPIO_PIN_5
#define M2_GPIO_Port GPIOA
#define M3_Pin GPIO_PIN_6
#define M3_GPIO_Port GPIOA
#define M4_Pin GPIO_PIN_7
#define M4_GPIO_Port GPIOA
#define M1_Pin GPIO_PIN_0
#define M1_GPIO_Port GPIOB
#define M5_Pin GPIO_PIN_1
#define M5_GPIO_Port GPIOB
#define M8_Pin GPIO_PIN_2
#define M8_GPIO_Port GPIOB
#define M7_Pin GPIO_PIN_10
#define M7_GPIO_Port GPIOB
#define COLON_Pin GPIO_PIN_12
#define COLON_GPIO_Port GPIOB
#define M6_Pin GPIO_PIN_13
#define M6_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_3
#define CE_GPIO_Port GPIOB
#define LBO_Pin GPIO_PIN_4
#define LBO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
