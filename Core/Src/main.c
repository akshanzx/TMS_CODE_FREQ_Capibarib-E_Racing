/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // Adicionado para permitir o uso do printf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch2;
DMA_HandleTypeDef hdma_tim2_ch4;
DMA_HandleTypeDef hdma_tim3_ch4;

/* USER CODE BEGIN PV */

float frequencias[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float Tensãos[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float Temperaturas[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// Filtro EMA: menor ALPHA = mais suavização, resposta mais lenta
// Sugerido: 0.05 (muito suave) a 0.3 (resposta rápida)
#define EMA_ALPHA 0.3f
float freq_filtrada[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

#define FREQ_MIN 1800.0f
#define FREQ_MAX 5450.0f
#define V_MIN    1.30f
#define V_MAX    2.44f

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint32_t in_capture_1[2];
volatile uint32_t in_capture_2[2];
volatile uint32_t in_capture_3[2];
volatile uint32_t in_capture_4[2];
volatile uint32_t in_capture_5[2];


/* ==== TABELA ==== */

float temp_table[] = {-40,-35,-30,-25,-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,
						45,50,55,60,65,70,75,80,85,90,95,100,105,110,115,120};

float Tensão_table[] = {2.44,2.42,2.40,2.38,2.35,2.32,2.27,2.23,2.17,2.11,2.05,1.99,1.92,1.86,1.80,1.74,1.68,
						1.63,1.59,1.55,1.51,1.48,1.45,1.43,1.40,1.38,1.37,1.35,1.34,1.33,1.32,1.31,1.30};



/* ==== INTERPOLAÇÃO ==== */

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* ==== FREQ -> Tensão ==== */

float freq_para_Tensão(float freq)
{
    if(freq < FREQ_MIN) freq = FREQ_MIN;
    if(freq > FREQ_MAX) freq = FREQ_MAX;
    return map_float(freq, FREQ_MIN, FREQ_MAX, V_MIN, V_MAX);
}

/* ==== Tensão -> TEMP ==== */

float Tensão_para_temp(float v)
{
    for(int i = 0; i < (sizeof(Tensão_table)/sizeof(float) - 1); i++)
    {
        if(v <= Tensão_table[i] && v >= Tensão_table[i+1])
        {
            return map_float(v,
                            Tensão_table[i], Tensão_table[i+1],
                            temp_table[i], temp_table[i+1]);
        }
    }
    return -999.0f;

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // --- CÓDIGO DE LEITURA DAS FREQUÊNCIAS ---
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Stop_DMA(&htim3, TIM_CHANNEL_4);
    HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_4);
    HAL_TIM_IC_Stop_DMA(&htim1, TIM_CHANNEL_1);

    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, in_capture_1, 2);   //  Porta 1
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, in_capture_2, 2);   //  Porta 2
    HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_4, in_capture_3, 2);   //  Porta 3
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, in_capture_4, 2);   //  Porta 4
    HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_1, in_capture_5, 2);   //  Porta 5

    HAL_Delay(250);

    // --- CÁLCULO DAS FREQUÊNCIAS  ---

    // Índice 0: Frequência 1 (Timer 2, Canal 1)
    {
        uint32_t delta1 = (in_capture_1[1] >= in_capture_1[0])
                        ? (in_capture_1[1] - in_capture_1[0])
                        : (65536U - in_capture_1[0] + in_capture_1[1]);
        if (delta1 > 0) {
            float freq_raw = 1000000.0f / (float)delta1;
            freq_filtrada[0] = EMA_ALPHA * freq_raw + (1.0f - EMA_ALPHA) * freq_filtrada[0];
            frequencias[0] = freq_filtrada[0];
            Tensãos[0] = freq_para_Tensão(frequencias[0]);
            Temperaturas[0] = Tensão_para_temp(Tensãos[0]);
        }
    }

    // Índice 1: Frequência 2 (Timer 2, Canal 2)
    {
        uint32_t delta2 = (in_capture_2[1] >= in_capture_2[0])
                        ? (in_capture_2[1] - in_capture_2[0])
                        : (65536U - in_capture_2[0] + in_capture_2[1]);
        if (delta2 > 0) {
            float freq_raw = 1000000.0f / (float)delta2;
            freq_filtrada[1] = EMA_ALPHA * freq_raw + (1.0f - EMA_ALPHA) * freq_filtrada[1];
            frequencias[1] = freq_filtrada[1];
            Tensãos[1] = freq_para_Tensão(frequencias[1]);
            Temperaturas[1] = Tensão_para_temp(Tensãos[1]);
        }
    }

    // Índice 2: Frequência 3 (Timer 15, Canal 1)
    {
        uint32_t delta3 = (in_capture_3[1] >= in_capture_3[0])
                        ? (in_capture_3[1] - in_capture_3[0])
                        : (65536U - in_capture_3[0] + in_capture_3[1]);
        if (delta3 > 0) {
            float freq_raw = 1000000.0f / (float)delta3;
            freq_filtrada[2] = EMA_ALPHA * freq_raw + (1.0f - EMA_ALPHA) * freq_filtrada[2];
            frequencias[2] = freq_filtrada[2];
            Tensãos[2] = freq_para_Tensão(frequencias[2]);
            Temperaturas[2] = Tensão_para_temp(Tensãos[2]);
        }
    }

    // Índice 3: Frequência 4 (Timer 1, Canal 4)
    {
        uint32_t delta4 = (in_capture_4[1] >= in_capture_4[0])
                        ? (in_capture_4[1] - in_capture_4[0])
                        : (65536U - in_capture_4[0] + in_capture_4[1]);
        if (delta4 > 0) {
            float freq_raw = 1000000.0f / (float)delta4;
            freq_filtrada[3] = EMA_ALPHA * freq_raw + (1.0f - EMA_ALPHA) * freq_filtrada[3];
            frequencias[3] = freq_filtrada[3];
            Tensãos[3] = freq_para_Tensão(frequencias[3]);
            Temperaturas[3] = Tensão_para_temp(Tensãos[3]);
        }
    }

    // Índice 4: Frequência 5 (Timer 1, Canal 1)
    {
        uint32_t delta5 = (in_capture_5[1] >= in_capture_5[0])
                        ? (in_capture_5[1] - in_capture_5[0])
                        : (65536U - in_capture_5[0] + in_capture_5[1]);
        if (delta5 > 0) {
            float freq_raw = 1000000.0f / (float)delta5;
            freq_filtrada[4] = EMA_ALPHA * freq_raw + (1.0f - EMA_ALPHA) * freq_filtrada[4];
            frequencias[4] = freq_filtrada[4];
            Tensãos[4] = freq_para_Tensão(frequencias[4]);
            Temperaturas[4] = Tensão_para_temp(Tensãos[4]);
        }
    }


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Função de re-direcionamento para permitir que o printf envie dados
// para a porta SWO/ITM (Serial Wire Viewer) no STM32CubeIDE.
int _write(int file, char *ptr, int len)
{
  for (int i = 0; i < len; i++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
