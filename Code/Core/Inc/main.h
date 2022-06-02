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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENCODER0_REVERSE_Pin GPIO_PIN_2
#define ENCODER0_REVERSE_GPIO_Port GPIOE
#define ENCODER1_REVERSE_Pin GPIO_PIN_3
#define ENCODER1_REVERSE_GPIO_Port GPIOE
#define ENCODER2_REVERSE_Pin GPIO_PIN_13
#define ENCODER2_REVERSE_GPIO_Port GPIOC
#define TOUCH_CS_Pin GPIO_PIN_0
#define TOUCH_CS_GPIO_Port GPIOH
#define TOUCH_IRQ_Pin GPIO_PIN_1
#define TOUCH_IRQ_GPIO_Port GPIOH
#define SDCARD_CS_Pin GPIO_PIN_0
#define SDCARD_CS_GPIO_Port GPIOC
#define ANALOG0_Pin GPIO_PIN_0
#define ANALOG0_GPIO_Port GPIOA
#define ANALOG2_Pin GPIO_PIN_3
#define ANALOG2_GPIO_Port GPIOA
#define LED_LCD_ON_Pin GPIO_PIN_5
#define LED_LCD_ON_GPIO_Port GPIOA
#define ETH_RESET_Pin GPIO_PIN_0
#define ETH_RESET_GPIO_Port GPIOB
#define ANALOG1_Pin GPIO_PIN_1
#define ANALOG1_GPIO_Port GPIOB
#define INPUT2_Pin GPIO_PIN_2
#define INPUT2_GPIO_Port GPIOB
#define OUTPUT3_Pin GPIO_PIN_14
#define OUTPUT3_GPIO_Port GPIOB
#define OUTPUT2_Pin GPIO_PIN_15
#define OUTPUT2_GPIO_Port GPIOB
#define ENCODER2_M_Pin GPIO_PIN_12
#define ENCODER2_M_GPIO_Port GPIOD
#define ENCODER2_P_Pin GPIO_PIN_13
#define ENCODER2_P_GPIO_Port GPIOD
#define ENCODER0_M_Pin GPIO_PIN_8
#define ENCODER0_M_GPIO_Port GPIOA
#define ENCODER0_P_Pin GPIO_PIN_9
#define ENCODER0_P_GPIO_Port GPIOA
#define INPUT0_Pin GPIO_PIN_10
#define INPUT0_GPIO_Port GPIOA
#define INPUT1_Pin GPIO_PIN_15
#define INPUT1_GPIO_Port GPIOA
#define INPUT3_Pin GPIO_PIN_10
#define INPUT3_GPIO_Port GPIOC
#define LDAC_Pin GPIO_PIN_11
#define LDAC_GPIO_Port GPIOC
#define OUTPUT1_Pin GPIO_PIN_3
#define OUTPUT1_GPIO_Port GPIOD
#define OUTPUT0_Pin GPIO_PIN_6
#define OUTPUT0_GPIO_Port GPIOD
#define ENCODER1_M_Pin GPIO_PIN_4
#define ENCODER1_M_GPIO_Port GPIOB
#define ENCODER1_P_Pin GPIO_PIN_5
#define ENCODER1_P_GPIO_Port GPIOB
#define LCD_RESET_Pin GPIO_PIN_0
#define LCD_RESET_GPIO_Port GPIOE
#define CAMERA_RESET_Pin GPIO_PIN_1
#define CAMERA_RESET_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
