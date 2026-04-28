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

/* USER CODE BEGIN Private defines */

/*
 * MUX address pins: 4-bit binary address selects 1-of-16 channels.
 * All 5 segment MUXes share the same address bus — they switch in parallel.
 * Assigned to PA2-PA5 (free GPIO pins, same GPIOA bank as TIM2 inputs).
 * If you need different pins, change these defines AND update MX_GPIO_Init
 * (or let CubeMX regenerate after assigning them as GPIO_Output in the .ioc).
 */
/* MUX address bus moved to PB4-PB7 so that PA2/PA3 can be used by USART2
 * (ST-Link VCP).  Hardware: rewire PA2→PB4, PA3→PB5, PA4→PB6, PA5→PB7. */
#define MUX_S0_Pin        GPIO_PIN_4
#define MUX_S0_GPIO_Port  GPIOB
#define MUX_S1_Pin        GPIO_PIN_5
#define MUX_S1_GPIO_Port  GPIOB
#define MUX_S2_Pin        GPIO_PIN_6
#define MUX_S2_GPIO_Port  GPIOB
#define MUX_S3_Pin        GPIO_PIN_7
#define MUX_S3_GPIO_Port  GPIOB

/*
 * CAN pins: FDCAN1 on PB8 (RX, AF9) and PB9 (TX, AF9).
 * These are the standard FDCAN1 alternate-function pins on STM32G474.
 * PA11/PA12 are also valid but PA11 was previously TIM1_CH4 in some configs.
 */
#define CAN_RX_Pin        GPIO_PIN_8
#define CAN_RX_GPIO_Port  GPIOB
#define CAN_TX_Pin        GPIO_PIN_9
#define CAN_TX_GPIO_Port  GPIOB

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
