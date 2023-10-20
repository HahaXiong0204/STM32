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
#include "app.h"

#include "string.h"
#include "stdio.h"
#include "stdlib.h"


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
#define bom_red_Pin GPIO_PIN_4
#define bom_red_GPIO_Port GPIOE
#define bom_yellow_Pin GPIO_PIN_5
#define bom_yellow_GPIO_Port GPIOE
#define bom_green_Pin GPIO_PIN_6
#define bom_green_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_13
#define KEY2_GPIO_Port GPIOC
#define SPI5_SDA_Pin GPIO_PIN_6
#define SPI5_SDA_GPIO_Port GPIOF
#define WK_UP_Pin GPIO_PIN_0
#define WK_UP_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOH
#define KEY2H3_Pin GPIO_PIN_3
#define KEY2H3_GPIO_Port GPIOH
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define led0_pwm_Pin GPIO_PIN_1
#define led0_pwm_GPIO_Port GPIOB
#define shuchu1_Pin GPIO_PIN_8
#define shuchu1_GPIO_Port GPIOH
#define shuchu2_Pin GPIO_PIN_9
#define shuchu2_GPIO_Port GPIOH
#define shuchu3_Pin GPIO_PIN_10
#define shuchu3_GPIO_Port GPIOH
#define People_Pin GPIO_PIN_11
#define People_GPIO_Port GPIOH
#define machia_pwm_Pin GPIO_PIN_8
#define machia_pwm_GPIO_Port GPIOC
#define DHT11_Pin GPIO_PIN_14
#define DHT11_GPIO_Port GPIOG
#define machinery_speed_Pin GPIO_PIN_0
#define machinery_speed_GPIO_Port GPIOE
#define RC522_SDA_Pin GPIO_PIN_4
#define RC522_SDA_GPIO_Port GPIOI
#define RC522_RST_Pin GPIO_PIN_5
#define RC522_RST_GPIO_Port GPIOI

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
