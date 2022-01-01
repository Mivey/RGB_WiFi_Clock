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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define LAT2_Pin GPIO_PIN_1
#define LAT2_GPIO_Port GPIOC
#define SIN2_Pin GPIO_PIN_2
#define SIN2_GPIO_Port GPIOC
#define BLK2_Pin GPIO_PIN_3
#define BLK2_GPIO_Port GPIOC
#define RST_Pin GPIO_PIN_1
#define RST_GPIO_Port GPIOA
#define IO0_Pin GPIO_PIN_4
#define IO0_GPIO_Port GPIOA
#define IO0_EXTI_IRQn EXTI4_IRQn
#define LAT12_Pin GPIO_PIN_5
#define LAT12_GPIO_Port GPIOA
#define SIN12_Pin GPIO_PIN_7
#define SIN12_GPIO_Port GPIOA
#define BLK12_Pin GPIO_PIN_4
#define BLK12_GPIO_Port GPIOC
#define SLIDE_Pin GPIO_PIN_5
#define SLIDE_GPIO_Port GPIOC
#define LAT10_Pin GPIO_PIN_0
#define LAT10_GPIO_Port GPIOB
#define SIN10_Pin GPIO_PIN_10
#define SIN10_GPIO_Port GPIOB
#define BLK10_Pin GPIO_PIN_11
#define BLK10_GPIO_Port GPIOB
#define LAT8_Pin GPIO_PIN_13
#define LAT8_GPIO_Port GPIOB
#define SIN8_Pin GPIO_PIN_15
#define SIN8_GPIO_Port GPIOB
#define BLK8_Pin GPIO_PIN_6
#define BLK8_GPIO_Port GPIOC
#define FAULT_Pin GPIO_PIN_7
#define FAULT_GPIO_Port GPIOC
#define WC_Pin GPIO_PIN_9
#define WC_GPIO_Port GPIOA
#define VOUT1_EN_Pin GPIO_PIN_11
#define VOUT1_EN_GPIO_Port GPIOC
#define LAT6_Pin GPIO_PIN_2
#define LAT6_GPIO_Port GPIOD
#define SIN6_Pin GPIO_PIN_4
#define SIN6_GPIO_Port GPIOB
#define BLK6_Pin GPIO_PIN_5
#define BLK6_GPIO_Port GPIOB
#define LAT4_Pin GPIO_PIN_6
#define LAT4_GPIO_Port GPIOB
#define SIN4_Pin GPIO_PIN_8
#define SIN4_GPIO_Port GPIOB
#define BLK4_Pin GPIO_PIN_9
#define BLK4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
