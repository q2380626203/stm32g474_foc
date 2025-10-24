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
#include "stm32g4xx_hal.h"

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
#define ENBALE_Pin GPIO_PIN_14
#define ENBALE_GPIO_Port GPIOC
#define PWM_COTR_Pin GPIO_PIN_15
#define PWM_COTR_GPIO_Port GPIOC
#define SEN_SCLK_Pin GPIO_PIN_5
#define SEN_SCLK_GPIO_Port GPIOA
#define SEN_MOSI_Pin GPIO_PIN_7
#define SEN_MOSI_GPIO_Port GPIOA
#define DRV_CS_Pin GPIO_PIN_0
#define DRV_CS_GPIO_Port GPIOB
#define SEN_CS_Pin GPIO_PIN_2
#define SEN_CS_GPIO_Port GPIOB
#define PWM_INHC_Pin GPIO_PIN_8
#define PWM_INHC_GPIO_Port GPIOA
#define PWM_INHB_Pin GPIO_PIN_9
#define PWM_INHB_GPIO_Port GPIOA
#define PWM_INHA_Pin GPIO_PIN_10
#define PWM_INHA_GPIO_Port GPIOA
#define DRV_SCLK_Pin GPIO_PIN_10
#define DRV_SCLK_GPIO_Port GPIOC
#define DRV_MISO_Pin GPIO_PIN_11
#define DRV_MISO_GPIO_Port GPIOC
#define DRV_MOSI_Pin GPIO_PIN_12
#define DRV_MOSI_GPIO_Port GPIOC
#define SEN_MISO_Pin GPIO_PIN_4
#define SEN_MISO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
