/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define R2_Pin GPIO_PIN_13
#define R2_GPIO_Port GPIOC
#define L2_Pin GPIO_PIN_14
#define L2_GPIO_Port GPIOC
#define F_AIN1_Pin GPIO_PIN_0
#define F_AIN1_GPIO_Port GPIOA
#define F_AIN2_Pin GPIO_PIN_1
#define F_AIN2_GPIO_Port GPIOA
#define F_BIN1_Pin GPIO_PIN_2
#define F_BIN1_GPIO_Port GPIOA
#define F_BIN2_Pin GPIO_PIN_3
#define F_BIN2_GPIO_Port GPIOA
#define B_AIN1_Pin GPIO_PIN_4
#define B_AIN1_GPIO_Port GPIOA
#define B_AIN2_Pin GPIO_PIN_5
#define B_AIN2_GPIO_Port GPIOA
#define B_BIN1_Pin GPIO_PIN_6
#define B_BIN1_GPIO_Port GPIOA
#define B_BIN2_Pin GPIO_PIN_7
#define B_BIN2_GPIO_Port GPIOA
#define Echo_Pin GPIO_PIN_0
#define Echo_GPIO_Port GPIOB
#define Trig_Pin GPIO_PIN_1
#define Trig_GPIO_Port GPIOB
#define R_LED_Pin GPIO_PIN_12
#define R_LED_GPIO_Port GPIOB
#define L_LED_Pin GPIO_PIN_13
#define L_LED_GPIO_Port GPIOB
#define KEY_Pin GPIO_PIN_14
#define KEY_GPIO_Port GPIOB
#define KEY_EXTI_IRQn EXTI15_10_IRQn
#define R3_Pin GPIO_PIN_12
#define R3_GPIO_Port GPIOA
#define R1_Pin GPIO_PIN_4
#define R1_GPIO_Port GPIOB
#define L1_Pin GPIO_PIN_5
#define L1_GPIO_Port GPIOB
#define L3_Pin GPIO_PIN_7
#define L3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
