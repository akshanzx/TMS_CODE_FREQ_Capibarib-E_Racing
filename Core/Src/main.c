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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NUM_SENSORS     5
#define EMA_ALPHA       0.3f        // Filtro EMA: 0.05 (suave) a 0.3 (rápido)
#define FREQ_MIN        1800.0f     // Hz mínimo da V→F
#define FREQ_MAX        5450.0f     // Hz máximo da V→F
#define V_MIN           1.30f       // Tensão correspondente a FREQ_MAX
#define V_MAX           2.44f       // Tensão correspondente a FREQ_MIN
#define TIMER_CLOCK_HZ  1000000UL   // Timer a 1 MHz (prescaler 64-1 @ 64 MHz)
#define TIMER_OVERFLOW  65536U      // Contador de 16 bits
#define SAMPLE_DELAY_MS 250         // Janela de amostragem em ms

// ---- Modo de teste: gera PWM interno para validar o pipeline ----
// Descomente TEST_MODE para habilitar o gerador de frequência via TIM4 CH1 (PB6).
// Conecte PB6 com um jumper ao pino de entrada do segmento que quer testar.
#define TEST_MODE
#define TEST_FREQ_HZ_DEFAULT    3500U   // Frequência de teste inicial em Hz (1800–5450)

// ---- Modo de teste CAN: loopback interno (sem transceiver externo) ----
// Descomente CAN_LOOPBACK_TEST para o FDCAN conectar TX→RX internamente.
// O MCU transmite e recebe seus próprios frames; monitore can_rx_count e
// can_loopback_ok via Live Expressions para confirmar que o periférico funciona.
#define CAN_LOOPBACK_TEST

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

FDCAN_HandleTypeDef hfdcan1;

#ifdef CAN_LOOPBACK_TEST
// --- Diagnóstico de loopback CAN (Live Expressions) ---
// can_loopback_ok  : 1 = pelo menos 1 frame foi recebido de volta
// can_rx_count     : total de frames recebidos via loopback
// can_tx_errors    : nº de falhas em HAL_FDCAN_AddMessageToTxFifoQ
// can_error_code   : hfdcan1.ErrorCode — qualquer flag != 0 indica falha HAL
// can_rxf0s        : registrador RXF0S raw (bits[2:0] = nº de frames no FIFO0)
// can_txfqs        : registrador TXFQS raw (bits[4:0] = espaço livre no TX FIFO)
volatile uint32_t can_rx_count    = 0;
volatile uint8_t  can_loopback_ok = 0;
volatile uint32_t can_tx_errors   = 0;
volatile uint32_t can_error_code  = 0;
volatile uint32_t can_rxf0s       = 0;
volatile uint32_t can_txfqs       = 0;
#endif

#ifdef TEST_MODE
TIM_HandleTypeDef htim4_test;       // Handle do timer gerador de teste
// Volátil para permitir alteração via Live Expressions no debug.
// Não é 'static' nem 'const' para que o linker mantenha o símbolo visível.
volatile uint32_t test_freq_hz = TEST_FREQ_HZ_DEFAULT;
#endif

// Buffers DMA: dois timestamps de borda de subida por canal
volatile uint32_t in_capture_1[2];
volatile uint32_t in_capture_2[2];
volatile uint32_t in_capture_3[2];
volatile uint32_t in_capture_4[2];
volatile uint32_t in_capture_5[2];

// Resultados de cada segmento (índices 0–4)
float frequencias[NUM_SENSORS];
float Tensoes[NUM_SENSORS];
float Temperaturas[NUM_SENSORS];
float freq_filtrada[NUM_SENSORS];

// Tabela de conversão Tensão → Temperatura (-40°C a +120°C, passo 5°C)
static const float temp_table[] = {
    -40, -35, -30, -25, -20, -15, -10,  -5,   0,   5,  10,  15,  20,
     25,  30,  35,  40,  45,  50,  55,  60,  65,  70,  75,  80,  85,
     90,  95, 100, 105, 110, 115, 120
};

static const float tensao_table[] = {
    2.44, 2.42, 2.40, 2.38, 2.35, 2.32, 2.27, 2.23, 2.17, 2.11, 2.05, 1.99, 1.92,
    1.86, 1.80, 1.74, 1.68, 1.63, 1.59, 1.55, 1.51, 1.48, 1.45, 1.43, 1.40, 1.38,
    1.37, 1.35, 1.34, 1.33, 1.32, 1.31, 1.30
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

static float map_float(float x, float in_min, float in_max, float out_min, float out_max);
static float freq_para_tensao(float freq);
static float tensao_para_temp(float v);
static void processar_canal(int idx, volatile uint32_t *capture);
static void MX_FDCAN1_Init(void);
static HAL_StatusTypeDef send_address_claim(void);
static HAL_StatusTypeDef send_thermistor_summary(int8_t minT, int8_t maxT, int8_t avgT,
                                                  uint8_t count, uint8_t id_max, uint8_t id_min);
static HAL_StatusTypeDef send_all_temps_to_esp(int8_t temps[NUM_SENSORS]);
#ifdef TEST_MODE
static void test_pwm_start(uint32_t freq_hz);
static void test_pwm_update(uint32_t freq_hz);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Interpolação linear genérica
static float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Frequência (Hz) → Tensão (V), com clamp nos limites
static float freq_para_tensao(float freq)
{
    if (freq < FREQ_MIN) freq = FREQ_MIN;
    if (freq > FREQ_MAX) freq = FREQ_MAX;
    return map_float(freq, FREQ_MIN, FREQ_MAX, V_MIN, V_MAX);
}

// Tensão (V) → Temperatura (°C) por interpolação na tabela
static float tensao_para_temp(float v)
{
    int n = sizeof(tensao_table) / sizeof(float);
    for (int i = 0; i < n - 1; i++)
    {
        if (v <= tensao_table[i] && v >= tensao_table[i + 1])
        {
            return map_float(v,
                             tensao_table[i], tensao_table[i + 1],
                             temp_table[i],   temp_table[i + 1]);
        }
    }
    return -999.0f; // Fora do range da tabela
}

// Processa um canal: calcula frequência, aplica EMA e converte para temperatura
static void processar_canal(int idx, volatile uint32_t *capture)
{
    uint32_t delta = (capture[1] >= capture[0])
                   ? (capture[1] - capture[0])
                   : (TIMER_OVERFLOW - capture[0] + capture[1]);

    if (delta == 0) return;

    float freq_raw = (float)TIMER_CLOCK_HZ / (float)delta;
    freq_filtrada[idx] = EMA_ALPHA * freq_raw + (1.0f - EMA_ALPHA) * freq_filtrada[idx];
    frequencias[idx]   = freq_filtrada[idx];
    Tensoes[idx]       = freq_para_tensao(frequencias[idx]);
    Temperaturas[idx]  = tensao_para_temp(Tensoes[idx]);
}

#ifdef TEST_MODE
// Gera um sinal PWM 50% duty em TIM4 CH1 (PB6) para testar o pipeline V→F→T.
// Jumper PB6 → pino de entrada do segmento que quer validar.
static void test_pwm_start(uint32_t freq_hz)
{
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // PB6 como AF2 (TIM4_CH1)
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_6;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);

    // ARR para atingir a frequência desejada com timer a 1 MHz
    uint32_t arr = (TIMER_CLOCK_HZ / freq_hz) - 1U;

    htim4_test.Instance               = TIM4;
    htim4_test.Init.Prescaler         = 64 - 1;      // 64 MHz / 64 = 1 MHz
    htim4_test.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4_test.Init.Period            = arr;
    htim4_test.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4_test.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim4_test) != HAL_OK)
    {
        Error_Handler();
    }

    // Canal 1: PWM1, 50% duty
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = (arr + 1U) / 2U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4_test, &oc, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_PWM_Start(&htim4_test, TIM_CHANNEL_1);
}

// Atualiza em tempo real a frequência gerada por TIM4 CH1, sem reiniciar o timer.
// Permite alterar test_freq_hz via Live Expressions no STM32CubeIDE.
static void test_pwm_update(uint32_t freq_hz)
{
    if (freq_hz == 0U) return;
    uint32_t arr = (TIMER_CLOCK_HZ / freq_hz) - 1U;
    __HAL_TIM_SET_AUTORELOAD(&htim4_test, arr);
    __HAL_TIM_SET_COMPARE(&htim4_test, TIM_CHANNEL_1, (arr + 1U) / 2U);
}
#endif

// Inicializa FDCAN1 em 500 kbps (Classic CAN) nos pinos PB8=RX, PB9=TX (AF9).
// Bit timing @ 64 MHz: Prescaler=8, Seg1=12, Seg2=3 → 16 tq × 125 ns = 2 µs = 500 kbps.
static void MX_FDCAN1_Init(void)
{
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Seleciona PCLK1 (64 MHz) como clock kernel do FDCAN.
    // O reset default é HSE (0x0), que não está habilitado neste projeto (HSI+PLL).
    // Sem isso, o FDCAN não tem clock de bit timing → TX FIFO trava cheio.
    __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PCLK1);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;   // PB8=RX, PB9=TX
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &gpio);

    hfdcan1.Instance                  = FDCAN1;
    hfdcan1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    // CAN_LOOPBACK_TEST: TX→RX internamente, sem transceiver nem barramento externo.
    // Normal: barramento físico nos pinos PB8(RX)/PB9(TX).
#ifdef CAN_LOOPBACK_TEST
    // External Loopback: TX opera normalmente (bits dominantes OK) + TX conectado
    // internamente ao RX. Internal Loopback (MON=1) trava TX FIFO — não usar.
    hfdcan1.Init.Mode                 = FDCAN_MODE_EXTERNAL_LOOPBACK;
#else
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
#endif
    hfdcan1.Init.AutoRetransmission   = DISABLE;
    hfdcan1.Init.TransmitPause        = DISABLE;
    hfdcan1.Init.ProtocolException    = DISABLE;
    hfdcan1.Init.NominalPrescaler     = 8;
    hfdcan1.Init.NominalSyncJumpWidth = 3;
    hfdcan1.Init.NominalTimeSeg1      = 12;
    hfdcan1.Init.NominalTimeSeg2      = 3;
    hfdcan1.Init.DataPrescaler        = 1;
    hfdcan1.Init.DataSyncJumpWidth    = 1;
    hfdcan1.Init.DataTimeSeg1         = 1;
    hfdcan1.Init.DataTimeSeg2         = 1;
#ifdef CAN_LOOPBACK_TEST
    hfdcan1.Init.StdFiltersNbr        = 1;   // aceita standard IDs (0x350)
    hfdcan1.Init.ExtFiltersNbr        = 1;   // aceita extended IDs (0x1839F380)
#else
    hfdcan1.Init.StdFiltersNbr        = 0;
    hfdcan1.Init.ExtFiltersNbr        = 0;
#endif
    hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

#ifdef CAN_LOOPBACK_TEST
    // Filtro extended (máscara 0 = aceita qualquer 29-bit ID) → FIFO0
    FDCAN_FilterTypeDef sf = {0};
    sf.IdType       = FDCAN_EXTENDED_ID;
    sf.FilterIndex  = 0;
    sf.FilterType   = FDCAN_FILTER_MASK;
    sf.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sf.FilterID1    = 0x00000000U;
    sf.FilterID2    = 0x00000000U;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sf) != HAL_OK) { Error_Handler(); }

    // Filtro standard (máscara 0 = aceita qualquer 11-bit ID) → FIFO0
    sf.IdType       = FDCAN_STANDARD_ID;
    sf.FilterIndex  = 0;
    sf.FilterID1    = 0x000U;
    sf.FilterID2    = 0x000U;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sf) != HAL_OK) { Error_Handler(); }

    // Frames sem filtro também vão para FIFO0
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_FILTER_REMOTE,      FDCAN_FILTER_REMOTE);
#endif
}

// Envia Address Claim J1939 (CAN ID 0x18EEFF80, 200 ms conforme protocolo Orion TEM).
// Source Address 0x80 = módulo #0; alterar o último byte do ID para múltiplos módulos.
static HAL_StatusTypeDef send_address_claim(void)
{
    FDCAN_TxHeaderTypeDef txh = {0};
    uint8_t data[8] = {0xF3, 0x00, 0x80, 0x00, 0x40, 0x1E, 0x90, 0x00};

    txh.Identifier         = 0x18EEFF80U;
    txh.IdType             = FDCAN_EXTENDED_ID;
    txh.TxFrameType        = FDCAN_DATA_FRAME;
    txh.DataLength         = FDCAN_DLC_BYTES_8;
    txh.ErrorStateIndicator= FDCAN_ESI_ACTIVE;
    txh.BitRateSwitch      = FDCAN_BRS_OFF;
    txh.FDFormat           = FDCAN_CLASSIC_CAN;
    txh.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txh.MessageMarker      = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txh, data);
}

// Envia resumo de temperatura ao Orion BMS (CAN ID 0x1839F380, 100 ms).
// Payload: [reservado, minT, maxT, avgT, count, id_max, id_min, checksum].
// Checksum = (0x39 + 8 + soma dos 7 bytes anteriores) & 0xFF.
static HAL_StatusTypeDef send_thermistor_summary(int8_t minT, int8_t maxT, int8_t avgT,
                                                  uint8_t count, uint8_t id_max, uint8_t id_min)
{
    FDCAN_TxHeaderTypeDef txh = {0};
    uint8_t data[8];

    data[0] = 0;
    data[1] = (uint8_t)minT;
    data[2] = (uint8_t)maxT;
    data[3] = (uint8_t)avgT;
    data[4] = count;
    data[5] = id_max;
    data[6] = id_min;

    uint16_t cksum = 0x39U + 8U;
    for (int i = 0; i < 7; i++) cksum += data[i];
    data[7] = (uint8_t)(cksum & 0xFFU);

    txh.Identifier         = 0x1839F380U;
    txh.IdType             = FDCAN_EXTENDED_ID;
    txh.TxFrameType        = FDCAN_DATA_FRAME;
    txh.DataLength         = FDCAN_DLC_BYTES_8;
    txh.ErrorStateIndicator= FDCAN_ESI_ACTIVE;
    txh.BitRateSwitch      = FDCAN_BRS_OFF;
    txh.FDFormat           = FDCAN_CLASSIC_CAN;
    txh.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txh.MessageMarker      = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txh, data);
}

// Envia os 5 valores de temperatura ao ESP (CAN ID 0x350, standard 11-bit).
// Bytes 0–4 = temperaturas dos segmentos 1–5 (int8_t); bytes 5–7 = 0.
static HAL_StatusTypeDef send_all_temps_to_esp(int8_t temps[NUM_SENSORS])
{
    FDCAN_TxHeaderTypeDef txh = {0};
    uint8_t data[8] = {0};

    for (int i = 0; i < NUM_SENSORS; i++)
        data[i] = (uint8_t)temps[i];

    txh.Identifier         = 0x350U;
    txh.IdType             = FDCAN_STANDARD_ID;
    txh.TxFrameType        = FDCAN_DATA_FRAME;
    txh.DataLength         = FDCAN_DLC_BYTES_8;
    txh.ErrorStateIndicator= FDCAN_ESI_ACTIVE;
    txh.BitRateSwitch      = FDCAN_BRS_OFF;
    txh.FDFormat           = FDCAN_CLASSIC_CAN;
    txh.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txh.MessageMarker      = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txh, data);
}

// Redireciona printf para SWO/ITM (Serial Wire Viewer)
int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
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

  MX_FDCAN1_Init();
  HAL_FDCAN_Start(&hfdcan1);
  (void)send_address_claim();   // Address Claim inicial ao energizar

#ifdef TEST_MODE
  test_pwm_start(test_freq_hz);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#ifdef TEST_MODE
  uint32_t last_test_freq_hz = test_freq_hz;
#endif
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#ifdef TEST_MODE
    // Aplica qualquer alteração feita em test_freq_hz via Live Expressions
    if (test_freq_hz != last_test_freq_hz)
    {
        test_pwm_update(test_freq_hz);
        last_test_freq_hz = test_freq_hz;
    }
#endif

    // Reinicia contadores e recomeça captura DMA em todos os canais
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Stop_DMA(&htim3, TIM_CHANNEL_4);
    HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_4);
    HAL_TIM_IC_Stop_DMA(&htim1, TIM_CHANNEL_1);

    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, in_capture_1, 2); // Segmento 1
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, in_capture_2, 2); // Segmento 2
    HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_4, in_capture_3, 2); // Segmento 3
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, in_capture_4, 2); // Segmento 4
    HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_1, in_capture_5, 2); // Segmento 5

    HAL_Delay(SAMPLE_DELAY_MS);

    // Calcula frequência, filtra e converte para temperatura
    processar_canal(0, in_capture_1);
    processar_canal(1, in_capture_2);
    processar_canal(2, in_capture_3);
    processar_canal(3, in_capture_4);
    processar_canal(4, in_capture_5);

    // Calcula estatísticas e envia via CAN
    {
        int8_t temps_i8[NUM_SENSORS];
        int8_t minT = 127, maxT = -128;
        uint8_t id_min = 0, id_max = 0;
        float soma = 0.0f;

        for (int i = 0; i < NUM_SENSORS; i++)
        {
            float t = Temperaturas[i];
            int8_t ti = (t > 127.0f) ? 127 : (t < -128.0f) ? -128 : (int8_t)t;
            temps_i8[i] = ti;
            soma += t;
            if (ti < minT) { minT = ti; id_min = (uint8_t)i; }
            if (ti > maxT) { maxT = ti; id_max = (uint8_t)i; }
        }
        int8_t avgT = (int8_t)(soma / NUM_SENSORS);

#ifdef CAN_LOOPBACK_TEST
        if (send_address_claim()    != HAL_OK) can_tx_errors++;
        HAL_Delay(10);
        if (send_thermistor_summary(minT, maxT, avgT, NUM_SENSORS, id_max, id_min) != HAL_OK) can_tx_errors++;
        if (send_all_temps_to_esp(temps_i8) != HAL_OK) can_tx_errors++;
        HAL_Delay(5);   // garante que os 3 frames (~390 µs) já chegaram ao FIFO0

        // Snapshot dos registradores para diagnóstico via Live Expressions
        can_error_code = hfdcan1.ErrorCode;
        can_rxf0s      = FDCAN1->RXF0S;
        can_txfqs      = FDCAN1->TXFQS;

        {
            FDCAN_RxHeaderTypeDef rxh;
            uint8_t rxbuf[8];
            while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
            {
                HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxh, rxbuf);
                can_rx_count++;
                can_loopback_ok = 1;
            }
        }
#else
        send_address_claim();
        HAL_Delay(10);
        send_thermistor_summary(minT, maxT, avgT, NUM_SENSORS, id_max, id_min);
        send_all_temps_to_esp(temps_i8);
#endif
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
