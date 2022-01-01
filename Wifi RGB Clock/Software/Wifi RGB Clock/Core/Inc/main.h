/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define Latch_2_Pin GPIO_PIN_1
#define Latch_2_GPIO_Port GPIOC
#define Sin_2_Pin GPIO_PIN_2
#define Sin_2_GPIO_Port GPIOC
#define Blank_2_Pin GPIO_PIN_3
#define Blank_2_GPIO_Port GPIOC
#define Clock_6_Pin GPIO_PIN_0
#define Clock_6_GPIO_Port GPIOA
#define Latch_12_Pin GPIO_PIN_5
#define Latch_12_GPIO_Port GPIOA
#define Clock_12_Pin GPIO_PIN_6
#define Clock_12_GPIO_Port GPIOA
#define Sin_12_Pin GPIO_PIN_7
#define Sin_12_GPIO_Port GPIOA
#define Blank_12_Pin GPIO_PIN_4
#define Blank_12_GPIO_Port GPIOC
#define Slide_Pin GPIO_PIN_5
#define Slide_GPIO_Port GPIOC
#define Latch_10_Pin GPIO_PIN_0
#define Latch_10_GPIO_Port GPIOB
#define Clock_10_Pin GPIO_PIN_1
#define Clock_10_GPIO_Port GPIOB
#define Sin_10_Pin GPIO_PIN_10
#define Sin_10_GPIO_Port GPIOB
#define Blank_10_Pin GPIO_PIN_11
#define Blank_10_GPIO_Port GPIOB
#define Latch_8_Pin GPIO_PIN_13
#define Latch_8_GPIO_Port GPIOB
#define Sin_8_Pin GPIO_PIN_15
#define Sin_8_GPIO_Port GPIOB
#define Blank_8_Pin GPIO_PIN_6
#define Blank_8_GPIO_Port GPIOC
#define Fault_Pin GPIO_PIN_7
#define Fault_GPIO_Port GPIOC
#define Latch_6_Pin GPIO_PIN_8
#define Latch_6_GPIO_Port GPIOC
#define Vout1_En_Pin GPIO_PIN_10
#define Vout1_En_GPIO_Port GPIOA
#define Sin_6_Pin GPIO_PIN_11
#define Sin_6_GPIO_Port GPIOC
#define Blank_6_Pin GPIO_PIN_2
#define Blank_6_GPIO_Port GPIOD
#define UP_Pin GPIO_PIN_3
#define UP_GPIO_Port GPIOB
#define Select_Pin GPIO_PIN_4
#define Select_GPIO_Port GPIOB
#define Down_Pin GPIO_PIN_5
#define Down_GPIO_Port GPIOB
#define Latch_4_Pin GPIO_PIN_6
#define Latch_4_GPIO_Port GPIOB
#define Clock_4_Pin GPIO_PIN_7
#define Clock_4_GPIO_Port GPIOB
#define Sin_4_Pin GPIO_PIN_8
#define Sin_4_GPIO_Port GPIOB
#define Blank_4_Pin GPIO_PIN_9
#define Blank_4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
