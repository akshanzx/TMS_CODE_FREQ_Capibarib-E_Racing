/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32g4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_tim1_ch1;

extern DMA_HandleTypeDef hdma_tim2_ch1;

extern DMA_HandleTypeDef hdma_tim2_ch2;

extern DMA_HandleTypeDef hdma_tim2_ch4;

extern DMA_HandleTypeDef hdma_tim3_ch4;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  HAL_PWREx_DisableUCPDDeadBattery();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
  * @brief TIM_Base MSP Initialization
  * This function configures the hardware resources used in this example
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_base->Instance==TIM1)
  {
    /* USER CODE BEGIN TIM1_MspInit 0 */

    /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM1 DMA Init */
    /* TIM1_CH1 Init */
    hdma_tim1_ch1.Instance = DMA1_Channel1;
    hdma_tim1_ch1.Init.Request = DMA_REQUEST_TIM1_CH1;
    hdma_tim1_ch1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim1_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim1_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC1],hdma_tim1_ch1);

    /* USER CODE BEGIN TIM1_MspInit 1 */

    /* USER CODE END TIM1_MspInit 1 */
  }
  else if(htim_base->Instance==TIM2)
  {
    /* USER CODE BEGIN TIM2_MspInit 0 */

    /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    PB11     ------> TIM2_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* TIM2 DMA Init */
    /* TIM2_CH1 Init */
    hdma_tim2_ch1.Instance = DMA1_Channel4;
    hdma_tim2_ch1.Init.Request = DMA_REQUEST_TIM2_CH1;
    hdma_tim2_ch1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC1],hdma_tim2_ch1);

    /* TIM2_CH2 Init */
    hdma_tim2_ch2.Instance = DMA1_Channel5;
    hdma_tim2_ch2.Init.Request = DMA_REQUEST_TIM2_CH2;
    hdma_tim2_ch2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim2_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch2.Init.Mode = DMA_NORMAL;
    hdma_tim2_ch2.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim2_ch2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC2],hdma_tim2_ch2);

    /* TIM2_CH4 Init */
    hdma_tim2_ch4.Instance = DMA1_Channel2;
    hdma_tim2_ch4.Init.Request = DMA_REQUEST_TIM2_CH4;
    hdma_tim2_ch4.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim2_ch4.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch4.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch4.Init.Mode = DMA_NORMAL;
    hdma_tim2_ch4.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim2_ch4) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC4],hdma_tim2_ch4);

    /* USER CODE BEGIN TIM2_MspInit 1 */

    /* USER CODE END TIM2_MspInit 1 */
  }
  else if(htim_base->Instance==TIM3)
  {
    /* USER CODE BEGIN TIM3_MspInit 0 */

    /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PB1     ------> TIM3_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* TIM3 DMA Init */
    /* TIM3_CH4 Init */
    hdma_tim3_ch4.Instance = DMA1_Channel6;
    hdma_tim3_ch4.Init.Request = DMA_REQUEST_TIM3_CH4;
    hdma_tim3_ch4.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim3_ch4.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim3_ch4.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim3_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim3_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim3_ch4.Init.Mode = DMA_NORMAL;
    hdma_tim3_ch4.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim3_ch4) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC4],hdma_tim3_ch4);

    /* USER CODE BEGIN TIM3_MspInit 1 */

    /* USER CODE END TIM3_MspInit 1 */
  }

}

/**
  * @brief TIM_Base MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
    /* USER CODE BEGIN TIM1_MspDeInit 0 */

    /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

    /* TIM1 DMA DeInit */
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC1]);
    /* USER CODE BEGIN TIM1_MspDeInit 1 */

    /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM2)
  {
    /* USER CODE BEGIN TIM2_MspDeInit 0 */

    /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    PB11     ------> TIM2_CH4
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

    /* TIM2 DMA DeInit */
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC2]);
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC4]);
    /* USER CODE BEGIN TIM2_MspDeInit 1 */

    /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM3)
  {
    /* USER CODE BEGIN TIM3_MspDeInit 0 */

    /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /**TIM3 GPIO Configuration
    PB1     ------> TIM3_CH4
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);

    /* TIM3 DMA DeInit */
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC4]);
    /* USER CODE BEGIN TIM3_MspDeInit 1 */

    /* USER CODE END TIM3_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/**
 * @brief  FDCAN MSP Initialization — called by HAL_FDCAN_Init().
 *         Configures PB8 (FDCAN1_RX, AF9) and PB9 (FDCAN1_TX, AF9),
 *         enables the FDCAN peripheral clock, and enables the NVIC lines
 *         needed for the TX FIFO (not strictly required if we never use
 *         TX interrupts, but good practice to have them available).
 */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hfdcan->Instance == FDCAN1)
    {
        /* FDCAN1 peripheral clock */
        __HAL_RCC_FDCAN_CLK_ENABLE();

        /* GPIO clocks (GPIOB already enabled by TIM3 init, but safe to repeat) */
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /*
         * PB8 = FDCAN1_RX (AF9)
         * PB9 = FDCAN1_TX (AF9)
         */
        GPIO_InitStruct.Pin       = CAN_RX_Pin | CAN_TX_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(CAN_RX_GPIO_Port, &GPIO_InitStruct);
    }
}

/**
 * @brief  FDCAN MSP De-Initialization.
 */
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN1)
    {
        __HAL_RCC_FDCAN_CLK_DISABLE();
        HAL_GPIO_DeInit(CAN_RX_GPIO_Port, CAN_RX_Pin | CAN_TX_Pin);
    }
}

/* USER CODE END 1 */
