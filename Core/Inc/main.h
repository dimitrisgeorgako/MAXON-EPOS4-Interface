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
#define CAN2USB_IND_Pin GPIO_PIN_13
#define CAN2USB_IND_GPIO_Port GPIOC
#define CANOPEN_RED_Pin GPIO_PIN_14
#define CANOPEN_RED_GPIO_Port GPIOB
#define CANOPEN_GREEN_Pin GPIO_PIN_6
#define CANOPEN_GREEN_GPIO_Port GPIOC
#define USB_ENABLE_Pin GPIO_PIN_11
#define USB_ENABLE_GPIO_Port GPIOC
#define USB2CAN_IND_Pin GPIO_PIN_6
#define USB2CAN_IND_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define MAX_STEERING_MM		20
#define MIN_STEERING_MM		-20
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
