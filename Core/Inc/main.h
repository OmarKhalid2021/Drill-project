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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Load_Sensor_Pin GPIO_PIN_0
#define Load_Sensor_GPIO_Port GPIOA
#define Battery_average_Pin GPIO_PIN_1
#define Battery_average_GPIO_Port GPIOA
#define button_1_Pin GPIO_PIN_3
#define button_1_GPIO_Port GPIOA
#define button_1_EXTI_IRQn EXTI3_IRQn
#define button_2_Pin GPIO_PIN_4
#define button_2_GPIO_Port GPIOA
#define button_2_EXTI_IRQn EXTI4_IRQn
#define button_3_Pin GPIO_PIN_5
#define button_3_GPIO_Port GPIOA
#define button_3_EXTI_IRQn EXTI9_5_IRQn
#define button_4_Pin GPIO_PIN_6
#define button_4_GPIO_Port GPIOA
#define button_4_EXTI_IRQn EXTI9_5_IRQn
#define Buzzer_Pin GPIO_PIN_7
#define Buzzer_GPIO_Port GPIOA
#define PWM_control_Pin GPIO_PIN_8
#define PWM_control_GPIO_Port GPIOA
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
