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
#define EN_1V2_Pin GPIO_PIN_0
#define EN_1V2_GPIO_Port GPIOC
#define EN_ELVSS_Pin GPIO_PIN_1
#define EN_ELVSS_GPIO_Port GPIOC
#define QSPI_DO3_Pin GPIO_PIN_1
#define QSPI_DO3_GPIO_Port GPIOA
#define EN_IOVCC_Pin GPIO_PIN_4
#define EN_IOVCC_GPIO_Port GPIOC
#define EN_1V8_Pin GPIO_PIN_5
#define EN_1V8_GPIO_Port GPIOC
#define QSPI_SCK_Pin GPIO_PIN_1
#define QSPI_SCK_GPIO_Port GPIOB
#define QSPI_DO2_Pin GPIO_PIN_8
#define QSPI_DO2_GPIO_Port GPIOC
#define QSPI_DO0_Pin GPIO_PIN_9
#define QSPI_DO0_GPIO_Port GPIOC
#define QSPI_DO1_Pin GPIO_PIN_10
#define QSPI_DO1_GPIO_Port GPIOC
#define OLED_TE_Pin GPIO_PIN_4
#define OLED_TE_GPIO_Port GPIOB
#define OLED_TE_EXTI_IRQn EXTI4_IRQn
#define OLED_RST_Pin GPIO_PIN_5
#define OLED_RST_GPIO_Port GPIOB
#define QSPI_CSN_Pin GPIO_PIN_6
#define QSPI_CSN_GPIO_Port GPIOB
#define EN_3V3_Pin GPIO_PIN_9
#define EN_3V3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
