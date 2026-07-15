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
#define gray2_Pin GPIO_PIN_0
#define gray2_GPIO_Port GPIOC
#define gray3_Pin GPIO_PIN_1
#define gray3_GPIO_Port GPIOC
#define gray4_Pin GPIO_PIN_2
#define gray4_GPIO_Port GPIOC
#define gray5_Pin GPIO_PIN_3
#define gray5_GPIO_Port GPIOC
#define left_pwm_Pin GPIO_PIN_1
#define left_pwm_GPIO_Port GPIOA
#define right_pwm_Pin GPIO_PIN_2
#define right_pwm_GPIO_Port GPIOA
#define left_A_Pin GPIO_PIN_6
#define left_A_GPIO_Port GPIOA
#define left_B_Pin GPIO_PIN_7
#define left_B_GPIO_Port GPIOA
#define right_A_Pin GPIO_PIN_12
#define right_A_GPIO_Port GPIOD
#define right_B_Pin GPIO_PIN_13
#define right_B_GPIO_Port GPIOD
#define right_in2_Pin GPIO_PIN_8
#define right_in2_GPIO_Port GPIOC
#define left_in1_Pin GPIO_PIN_9
#define left_in1_GPIO_Port GPIOC
#define right_in1_Pin GPIO_PIN_12
#define right_in1_GPIO_Port GPIOC
#define left_in2_Pin GPIO_PIN_3
#define left_in2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
