/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define DEBUG_LED_Pin GPIO_PIN_3
#define DEBUG_LED_GPIO_Port GPIOA
#define DIG_INT1_Pin GPIO_PIN_4
#define DIG_INT1_GPIO_Port GPIOA
#define DIG_INT2_Pin GPIO_PIN_5
#define DIG_INT2_GPIO_Port GPIOA
#define DIG_INT3_Pin GPIO_PIN_6
#define DIG_INT3_GPIO_Port GPIOA
#define DIG_INT4_Pin GPIO_PIN_7
#define DIG_INT4_GPIO_Port GPIOA
#define DIG_INT5_Pin GPIO_PIN_4
#define DIG_INT5_GPIO_Port GPIOC
#define DIG_INT6_Pin GPIO_PIN_5
#define DIG_INT6_GPIO_Port GPIOC
#define DIG_INT7_Pin GPIO_PIN_0
#define DIG_INT7_GPIO_Port GPIOB
#define DIG_INT8_Pin GPIO_PIN_1
#define DIG_INT8_GPIO_Port GPIOB
#define DIG_INT9_Pin GPIO_PIN_2
#define DIG_INT9_GPIO_Port GPIOB
#define DIG_INT10_Pin GPIO_PIN_10
#define DIG_INT10_GPIO_Port GPIOB
#define DIG_INT11_Pin GPIO_PIN_11
#define DIG_INT11_GPIO_Port GPIOB
#define DIG_INT12_Pin GPIO_PIN_12
#define DIG_INT12_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
