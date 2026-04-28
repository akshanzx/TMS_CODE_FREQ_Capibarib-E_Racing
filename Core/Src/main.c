/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : TMS — Temperature Monitoring System
  *
  * ARCHITECTURE (new hardware):
  *   5 segments × 16 cells each = 80 cells total
  *
  *   Per segment:
  *     16 NTC sensors → 16-ch analog MUX → V-to-F converter
  *     → optical isolator → Timer Input Capture pin
  *
  *   MUX address (S0–S3): PA2–PA5, shared across all 5 segment MUXes.
  *   Selecting channel N on the address bus selects cell N on ALL 5 segments
  *   simultaneously, so all 5 V-to-F converters run in parallel per channel.
  *
  * SCAN CYCLE (every ~320 ms with default timing):
  *   for ch = 0..15:
  *     1. Drive MUX_S0..S3 to 'ch'
  *     2. Wait MUX_SETTLE_MS for V-to-F output to settle at new frequency
  *     3. Start DMA Input Capture on all 5 timer channels simultaneously
  *     4. Wait MUX_MEASURE_MS (captures 2 rising edges per channel)
  *     5. Compute frequency → voltage → temperature for each segment
  *
  * OUTPUTS after each full scan:
  *   CAN — 10 × classic CAN frames to Orion 2 BMS (80 temperatures)
  *          IDs 0x1839F380..0x1839F389  (29-bit extended)
  *          Encoding: data[byte] = (uint8_t)(temp_°C + 40)
  *          Msg 0: Seg0 cells 0-7 … Msg 9: Seg4 cells 8-15
  *
  *   CAN — 1 × summary frame to secondary MCU
  *          ID 0x100  (11-bit standard)
  *          Byte 0: T_min+40    Byte 1: min_cell  Byte 2: min_seg
  *          Byte 3: T_avg+40
  *          Byte 4: T_max+40    Byte 5: max_cell  Byte 6: max_seg
  *          Byte 7: 0x00 (reserved)
  *
  * HARDWARE PINS (do NOT change without updating CubeMX .ioc):
  *   PA0  TIM2_CH1  Segment 0 V-to-F input
  *   PA1  TIM2_CH2  Segment 1 V-to-F input
  *   PB1  TIM3_CH4  Segment 2 V-to-F input
  *   PB11 TIM2_CH4  Segment 3 V-to-F input
  *   PA8  TIM1_CH1  Segment 4 V-to-F input
  *   PA2  USART2_TX (AF7)  ST-Link VCP → PC dashboard (115200 baud)
  *   PA3  USART2_RX (AF7)  ST-Link VCP (unused, TX-only)
  *   PB4  GPIO_OUT  MUX_S0 (LSB of 4-bit channel address)
  *   PB5  GPIO_OUT  MUX_S1
  *   PB6  GPIO_OUT  MUX_S2
  *   PB7  GPIO_OUT  MUX_S3 (MSB)
  *   PB8  FDCAN1_RX (AF9)
  *   PB9  FDCAN1_TX (AF9)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---------- System dimensions ---------- */
#define NUM_SEGMENTS         1u    /* one V-to-F + isolator chain per segment */
#define NUM_CHANNELS        16u    /* 16-channel MUX per segment               */
#define NUM_CELLS           (NUM_SEGMENTS * NUM_CHANNELS)   /* 80 total        */

/* ---------- Scan timing (ms) ---------- */
/*
 * MUX_SETTLE_MS  : time from MUX address change until the V-to-F output is
 *                  stable at the new frequency.  Increase if your V-to-F
 *                  integrating capacitor is large (slow settling).
 * MUX_MEASURE_MS : measurement window after settling.  Must be > 2 × period
 *                  at the lowest expected frequency (1800 Hz → 1.1 ms),
 *                  so 10 ms gives a comfortable margin.
 */
#define MUX_SETTLE_MS        10u
#define MUX_MEASURE_MS       2u

/* ---------- V-to-F / temperature conversion ---------- */
#define EMA_ALPHA            0.3f   /* EMA smoothing: lower = smoother, slower */
#define FREQ_MIN          1800.0f   /* Hz  @ V_MIN (hottest cell) */
#define FREQ_MAX          5450.0f   /* Hz  @ V_MAX (coldest cell) */
#define V_MIN                1.30f  /* V */
#define V_MAX                2.44f  /* V */
#define TIMER_OVERFLOW     65536u   /* 16-bit timer ARR = 65535, wraps at 65536 */

/* ---------- Orion 2 BMS CAN protocol ----------
 *
 * The Orion 2 BMS receives cell temperatures from an external thermistor
 * module via CAN.  Ten 8-byte classic CAN frames carry all 80 readings:
 *
 *   Msg index  CAN ID           Cells
 *   ---------  ---------------  ----------------------
 *       0      ORION2_BASE+0    Seg 0, cells  0– 7
 *       1      ORION2_BASE+1    Seg 0, cells  8–15
 *       2      ORION2_BASE+2    Seg 1, cells  0– 7
 *       ...
 *       9      ORION2_BASE+9    Seg 4, cells  8–15
 *
 * Temperature encoding: data_byte = (uint8_t)(temp_°C + 40)
 *   Value 0  → −40 °C
 *   Value 40 →   0 °C
 *   Value 80 →  +40 °C
 *   Value 160→ +120 °C  (maximum in-range cell temperature)
 *
 * VERIFY ORION2_BASE_ID against the protocol table shown in your BMS
 * programming software — change the define below if your Orion 2 is
 * configured to listen on a different base address.
 */
#define ORION2_BASE_ID       0x1839F380u   /* 29-bit extended CAN ID */
#define ORION2_TEMP_OFFSET   40            /* encoding offset in °C  */
#define ORION2_NUM_MSGS      10u           /* 80 cells / 8 per frame */

/* ---------- Summary packet to secondary MCU ---------- */
#define SUMMARY_CAN_ID       0x100u        /* 11-bit standard CAN ID */

/* ---------- Modo de teste -------------------------------------------------------
 * Descomente TEST_MODE para habilitar o gerador de frequência via TIM16 CH1 (PA6).
 * Conecte PA6 com um jumper ao pino de entrada do segmento que quer testar (PA0).
 * O firmware opera normalmente (scan MUX, CAN, UART); a única adição é o PWM em PA6.
 *
 * test_freq_hz (Live Expressions) controla a frequência gerada em tempo real.
 * Faixa válida: 1800–5450 Hz.
 * -------------------------------------------------------------------------*/
#define TEST_MODE
#define TEST_FREQ_HZ_DEFAULT  4000U     /* frequência inicial em Hz */

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

/* ---- Live-expression variables (sempre presentes na RAM) ----
 * volatile: toda escrita vai direto para memória → Live Expressions sempre
 * mostra o valor atual, nunca uma cópia obsoleta de registrador.     */
volatile float     freq_filtrada[NUM_SEGMENTS][NUM_CHANNELS] __attribute__((used));
volatile float     temperatures[NUM_SEGMENTS][NUM_CHANNELS]  __attribute__((used));
volatile float     tensão[NUM_SEGMENTS][NUM_CHANNELS]  __attribute__((used));

#ifdef TEST_MODE
static TIM_HandleTypeDef htim16_test;
volatile uint32_t  test_freq_hz = TEST_FREQ_HZ_DEFAULT; /* Hz — altere via Live Expressions */
#endif

/* Voltage-to-temperature lookup table (monotonically decreasing voltage) */
static const float temp_table[] = {
    -40,-35,-30,-25,-20,-15,-10, -5,  0,  5, 10, 15, 20, 25,
     30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95,
    100,105,110,115,120
};
#define TEMP_TABLE_LEN  (sizeof(temp_table) / sizeof(float))

static const float voltage_table[] = {
    2.44f,2.42f,2.40f,2.38f,2.35f,2.32f,2.27f,2.23f,2.17f,2.11f,
    2.05f,1.99f,1.92f,1.86f,1.80f,1.74f,1.68f,1.63f,1.59f,1.55f,
    1.51f,1.48f,1.45f,1.43f,1.40f,1.38f,1.37f,1.35f,1.34f,1.33f,
    1.32f,1.31f,1.30f
};

/* Scan statistics — updated each full cycle */
static float t_min, t_avg, t_max;
static uint8_t min_cell, min_seg, max_cell, max_seg;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void FDCAN_Config(void);
static void USART2_Init(void);
static void USART2_Write(const char *buf, uint16_t len);
#ifdef TEST_MODE
static void test_pwm_start(uint32_t freq_hz);
static void test_pwm_update(uint32_t freq_hz);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* DMA capture buffers — two timestamps per channel (1 MHz timer → µs) */
static volatile uint32_t in_capture_1[2];  /* Seg 0: TIM2 CH1  (PA0)  */
static volatile uint32_t in_capture_2[2];  /* Seg 1: TIM2 CH2  (PA1)  */
static volatile uint32_t in_capture_3[2];  /* Seg 2: TIM3 CH4  (PB1)  */
static volatile uint32_t in_capture_4[2];  /* Seg 3: TIM2 CH4  (PB11) */
static volatile uint32_t in_capture_5[2];  /* Seg 4: TIM1 CH1  (PA8)  */

/* ---- Helper: linear interpolation ---- */
static float map_float(float x,
                       float in_min, float in_max,
                       float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* ---- Frequency (Hz) → analog voltage (V) ---- */
static float freq_to_voltage(float freq)
{
    if (freq < FREQ_MIN) freq = FREQ_MIN;
    if (freq > FREQ_MAX) freq = FREQ_MAX;
    return map_float(freq, FREQ_MIN, FREQ_MAX, V_MIN, V_MAX);
}

/* ---- Voltage (V) → temperature (°C) via lookup table ---- */
static float voltage_to_temp(float v)
{
    uint32_t i;
    for (i = 0; i < TEMP_TABLE_LEN - 1u; i++)
    {
        if (v <= voltage_table[i] && v >= voltage_table[i + 1u])
        {
            return map_float(v,
                             voltage_table[i], voltage_table[i + 1u],
                             temp_table[i],    temp_table[i + 1u]);
        }
    }
    return -999.0f;   /* out-of-range sentinel */
}

/* ---- MUX channel selection (S0–S3 → 4-bit binary address) ---- */
static void MUX_SetChannel(uint8_t ch)
{
    /* Drive each address bit individually; HAL_GPIO_WritePin is fast enough */
    HAL_GPIO_WritePin(MUX_S0_GPIO_Port, MUX_S0_Pin,
                      (ch & 0x01u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_S1_GPIO_Port, MUX_S1_Pin,
                      (ch & 0x02u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_S2_GPIO_Port, MUX_S2_Pin,
                      (ch & 0x04u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_S3_GPIO_Port, MUX_S3_Pin,
                      (ch & 0x08u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ---- Compute frequency from two DMA-captured timer timestamps ---- */
static float compute_frequency(volatile uint32_t *cap)
{
    uint32_t delta;
    if (cap[1] >= cap[0])
        delta = cap[1] - cap[0];
    else
        delta = TIMER_OVERFLOW - cap[0] + cap[1];   /* handle wrap-around */

    if (delta == 0u) return 0.0f;
    return 1000000.0f / (float)delta;               /* timer @ 1 MHz → µs */
}

/* ---- Compute min / avg / max across all 80 cells ---- */
static void ComputeStats(void)
{
    float sum = 0.0f;
    t_min = temperatures[0][0];
    t_max = temperatures[0][0];
    min_seg  = 0u; min_cell = 0u;
    max_seg  = 0u; max_cell = 0u;

    for (uint8_t seg = 0u; seg < NUM_SEGMENTS; seg++)
    {
        for (uint8_t ch = 0u; ch < NUM_CHANNELS; ch++)
        {
            float t = temperatures[seg][ch];
            sum += t;
            if (t < t_min) { t_min = t; min_seg = seg; min_cell = ch; }
            if (t > t_max) { t_max = t; max_seg = seg; max_cell = ch; }
        }
    }
    t_avg = sum / (float)NUM_CELLS;
}

/* ---- Encode temperature to Orion 2 byte format ---- */
static uint8_t encode_temp(float temp_c)
{
    int16_t raw = (int16_t)temp_c + ORION2_TEMP_OFFSET;
    if (raw < 0)   raw = 0;
    if (raw > 255) raw = 255;
    return (uint8_t)raw;
}

/* ---- Send 10 CAN frames to Orion 2 BMS ---- *
 *
 * Frame layout (10 messages × 8 bytes = 80 thermistor readings):
 *
 *   Msg  CAN ID           Seg  Cells
 *    0   ORION2_BASE+0     0   0– 7
 *    1   ORION2_BASE+1     0   8–15
 *    2   ORION2_BASE+2     1   0– 7
 *    3   ORION2_BASE+3     1   8–15
 *    4   ORION2_BASE+4     2   0– 7
 *    5   ORION2_BASE+5     2   8–15
 *    6   ORION2_BASE+6     3   0– 7
 *    7   ORION2_BASE+7     3   8–15
 *    8   ORION2_BASE+8     4   0– 7
 *    9   ORION2_BASE+9     4   8–15
 */
static void CAN_SendOrion2Data(void)
{
    FDCAN_TxHeaderTypeDef hdr;
    uint8_t data[8];

    hdr.IdType              = FDCAN_EXTENDED_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = FDCAN_DLC_BYTES_8;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch       = FDCAN_BRS_OFF;
    hdr.FDFormat            = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker       = 0u;

    uint8_t msg = 0u;
    for (uint8_t seg = 0u; seg < NUM_SEGMENTS; seg++)
    {
        /* Lower half: cells 0–7 */
        hdr.Identifier = ORION2_BASE_ID + msg;
        for (uint8_t b = 0u; b < 8u; b++)
            data[b] = encode_temp(temperatures[seg][b]);
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, data);
        msg++;

        /* Upper half: cells 8–15 */
        hdr.Identifier = ORION2_BASE_ID + msg;
        for (uint8_t b = 0u; b < 8u; b++)
            data[b] = encode_temp(temperatures[seg][8u + b]);
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, data);
        msg++;
    }
}

/* ---- Send 1 summary CAN frame to secondary MCU ---- *
 *
 *   Byte 0  T_min + 40  (uint8_t, same encoding as Orion 2 frames)
 *   Byte 1  min_cell    0–15  (MUX channel index of the coldest cell)
 *   Byte 2  min_seg     0–4   (segment of the coldest cell)
 *   Byte 3  T_avg + 40  (uint8_t)
 *   Byte 4  T_max + 40  (uint8_t)
 *   Byte 5  max_cell    0–15  (MUX channel index of the hottest cell)
 *   Byte 6  max_seg     0–4   (segment of the hottest cell)
 *   Byte 7  0x00        reserved
 */
static void CAN_SendSummaryPacket(void)
{
    FDCAN_TxHeaderTypeDef hdr;
    uint8_t data[8];

    hdr.Identifier          = SUMMARY_CAN_ID;
    hdr.IdType              = FDCAN_STANDARD_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = FDCAN_DLC_BYTES_8;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch       = FDCAN_BRS_OFF;
    hdr.FDFormat            = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker       = 0u;

    data[0] = encode_temp(t_min);
    data[1] = min_cell;
    data[2] = min_seg;
    data[3] = encode_temp(t_avg);
    data[4] = encode_temp(t_max);
    data[5] = max_cell;
    data[6] = max_seg;
    data[7] = 0x00u;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, data);
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

  /* Configure FDCAN1 for 500 kbps (calls HAL_FDCAN_MspInit → GPIO + clock) */
  FDCAN_Config();

  /* USART2 via ST-Link VCP — PA2=TX, PA3=RX, 115200 baud */
  USART2_Init();

#ifdef TEST_MODE
  test_pwm_start(test_freq_hz);
#endif

  /* Drive MUX to channel 0 and wait one full settle+measure cycle before
     entering the main loop, so the first scan begins with valid data. */
  MUX_SetChannel(0u);
  HAL_Delay(MUX_SETTLE_MS + MUX_MEASURE_MS);

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
    /* Aplica qualquer alteração feita em test_freq_hz via Live Expressions */
    if (test_freq_hz != last_test_freq_hz)
    {
        test_pwm_update(test_freq_hz);
        last_test_freq_hz = test_freq_hz;
    }
#endif

    /* ================================================================
     * FULL 16-CHANNEL MUX SCAN
     * For each MUX channel, all 5 segments are captured simultaneously.
     * ================================================================ */
    for (uint8_t ch = 0u; ch < NUM_CHANNELS; ch++)
    {
        /* 1. Select MUX channel on all 5 segment MUXes in parallel */
        MUX_SetChannel(ch);

        /* 2. Stop any previous DMA capture BEFORE settling so the V-to-F
              converter has the full settle window to stabilise at the new
              frequency with no active DMA that could capture residual edges
              from the previous channel. */
        HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);
        HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_2);
        HAL_TIM_IC_Stop_DMA(&htim3, TIM_CHANNEL_4);
        HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_4);
        HAL_TIM_IC_Stop_DMA(&htim1, TIM_CHANNEL_1);

        /* Reset timer counters so first-edge timestamps start near zero */
        __HAL_TIM_SET_COUNTER(&htim1, 0u);
        __HAL_TIM_SET_COUNTER(&htim2, 0u);
        __HAL_TIM_SET_COUNTER(&htim3, 0u);

        /* 3. Allow V-to-F converter output to settle at new frequency.
              No DMA is running during this window. */
        HAL_Delay(MUX_SETTLE_MS);

        /* 4. Zero buffers so stale data cannot bleed in if the DMA does not
              complete (channel with no signal). */
        in_capture_1[0] = in_capture_1[1] = 0u;
        in_capture_2[0] = in_capture_2[1] = 0u;
        in_capture_3[0] = in_capture_3[1] = 0u;
        in_capture_4[0] = in_capture_4[1] = 0u;
        in_capture_5[0] = in_capture_5[1] = 0u;

        /* 5. Arm DMA capture on all 5 channels simultaneously.
              Each DMA captures exactly 2 rising edges then stops. */
        HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)in_capture_1, 2u);
        HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t *)in_capture_2, 2u);
        HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t *)in_capture_3, 2u);
        HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, (uint32_t *)in_capture_4, 2u);
        HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)in_capture_5, 2u);

        /* Wait long enough to guarantee 2 edges at the lowest frequency.
           At 1800 Hz: period = 555 µs → 2 edges in ~1.1 ms.
           MUX_MEASURE_MS = 10 ms → very safe margin. */
        HAL_Delay(MUX_MEASURE_MS);

        /* 6. Compute frequency → voltage → temperature for all 5 segments */
        float freq_raw;

        /* Segment 0  (TIM2 CH1, PA0) */
        freq_raw = compute_frequency(in_capture_1);
        if (freq_raw > 0.0f)
        {
            freq_filtrada[0][ch] = EMA_ALPHA * freq_raw
                                 + (1.0f - EMA_ALPHA) * freq_filtrada[0][ch];
            temperatures[0][ch] = voltage_to_temp(
                                      freq_to_voltage(freq_filtrada[0][ch]));
            tensão[0][ch] = freq_to_voltage(freq_filtrada[0][ch]);
        }

        /* Segment 1  (TIM2 CH2, PA1) */
        freq_raw = compute_frequency(in_capture_2);
        if (freq_raw > 0.0f)
        {
            freq_filtrada[1][ch] = EMA_ALPHA * freq_raw
                                 + (1.0f - EMA_ALPHA) * freq_filtrada[1][ch];
            temperatures[1][ch] = voltage_to_temp(
                                      freq_to_voltage(freq_filtrada[1][ch]));
        }

        /* Segment 2  (TIM3 CH4, PB1) */
        freq_raw = compute_frequency(in_capture_3);
        if (freq_raw > 0.0f)
        {
            freq_filtrada[2][ch] = EMA_ALPHA * freq_raw
                                 + (1.0f - EMA_ALPHA) * freq_filtrada[2][ch];
            temperatures[2][ch] = voltage_to_temp(
                                      freq_to_voltage(freq_filtrada[2][ch]));
        }

        /* Segment 3  (TIM2 CH4, PB11) */
        freq_raw = compute_frequency(in_capture_4);
        if (freq_raw > 0.0f)
        {
            freq_filtrada[3][ch] = EMA_ALPHA * freq_raw
                                 + (1.0f - EMA_ALPHA) * freq_filtrada[3][ch];
            temperatures[3][ch] = voltage_to_temp(
                                      freq_to_voltage(freq_filtrada[3][ch]));
        }

        /* Segment 4  (TIM1 CH1, PA8) */
        freq_raw = compute_frequency(in_capture_5);
        if (freq_raw > 0.0f)
        {
            freq_filtrada[4][ch] = EMA_ALPHA * freq_raw
                                 + (1.0f - EMA_ALPHA) * freq_filtrada[4][ch];
            temperatures[4][ch] = voltage_to_temp(
                                      freq_to_voltage(freq_filtrada[4][ch]));
        }
    }

    /* ================================================================
     * AFTER FULL SCAN: compute stats and transmit CAN frames
     * ================================================================ */
    ComputeStats();
    CAN_SendOrion2Data();
    CAN_SendSummaryPacket();

    /* UART output to PC dashboard — one line per cell of segment 0:
     * Format: S<seg>,C<cell>,<freq_Hz>,<volt_V>,<temp_C>\r\n
     * Example: S0,C03,2480.50,1.840,36.12                        */
    {
        char    buf[64];
        int     len;
        for (uint8_t ch = 0u; ch < NUM_CHANNELS; ch++)
        {
            float freq = freq_filtrada[0][ch];
            float volt = freq_to_voltage(freq);
            len = snprintf(buf, sizeof(buf), "S0,C%02u,%.2f,%.3f,%.2f\r\n",
                           ch, freq, volt, temperatures[0][ch]);
            USART2_Write(buf, (uint16_t)len);
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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
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

  /*
   * MUX address outputs: PA2 (S0), PA3 (S1), PA4 (S2), PA5 (S3)
   * Configured here because they are user-added GPIOs not in the original
   * .ioc.  If you add them to CubeMX and regenerate, this block will be
   * replaced by the generated code — that is fine, just remove this block.
   */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin   = MUX_S0_Pin | MUX_S1_Pin | MUX_S2_Pin | MUX_S3_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Drive all address lines LOW (channel 0 selected by default) */
  HAL_GPIO_WritePin(GPIOB,
      MUX_S0_Pin | MUX_S1_Pin | MUX_S2_Pin | MUX_S3_Pin,
      GPIO_PIN_RESET);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Configure FDCAN1 for classic CAN at 500 kbps.
 *
 * Bit timing (FDCAN clock = 64 MHz, prescaler = 8):
 *   TQ duration = 8 / 64 MHz = 125 ns
 *   Bit time    = (1 + 13 + 2) × 125 ns = 2000 ns = 500 kbps
 *   Sample point = (1 + 13) / 16 = 87.5 %
 *
 * HAL_FDCAN_Init() calls HAL_FDCAN_MspInit() (defined in
 * stm32g4xx_hal_msp.c) which enables the FDCAN peripheral clock and
 * configures PB8/PB9 as FDCAN1_RX/TX with AF9.
 *
 * All incoming frames are rejected (we only transmit).
 */
static void FDCAN_Config(void)
{
    hfdcan1.Instance                  = FDCAN1;
    hfdcan1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission   = ENABLE;
    hfdcan1.Init.TransmitPause        = DISABLE;
    hfdcan1.Init.ProtocolException    = DISABLE;
    hfdcan1.Init.NominalPrescaler     = 8u;
    hfdcan1.Init.NominalSyncJumpWidth = 1u;
    hfdcan1.Init.NominalTimeSeg1      = 13u;
    hfdcan1.Init.NominalTimeSeg2      = 2u;
    /* Data phase fields unused for classic CAN but must be set */
    hfdcan1.Init.DataPrescaler        = 1u;
    hfdcan1.Init.DataSyncJumpWidth    = 1u;
    hfdcan1.Init.DataTimeSeg1         = 1u;
    hfdcan1.Init.DataTimeSeg2         = 1u;
    hfdcan1.Init.StdFiltersNbr        = 0u;
    hfdcan1.Init.ExtFiltersNbr        = 0u;
    hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
        Error_Handler();

    /* Reject all incoming frames — we only need to transmit */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
            FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
        Error_Handler();

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
        Error_Handler();
}

/* ---- USART2 helpers — register-level, no HAL_UART needed ----
 *
 * USART2 is connected to the ST-Link VCP on Nucleo-G474RE:
 *   PA2 = USART2_TX (AF7)  →  ST-Link RX
 *   PA3 = USART2_RX (AF7)  →  ST-Link TX  (unused, TX-only here)
 *
 * MUX address lines were moved from PA2-PA5 to PB4-PB7 to free these pins.
 */
static void USART2_Init(void)
{
    /* PA2 = AF7 (USART2_TX), PA3 = AF7 (USART2_RX — wired to ST-Link) */
    GPIOA->MODER   = (GPIOA->MODER   & ~(0xFu << 4u)) | (0xAu << 4u);  /* AF */
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(0xFu << 4u)) | (0x5u << 4u);  /* medium speed */
    GPIOA->PUPDR   = (GPIOA->PUPDR   & ~(0xFu << 4u));                  /* no pull */
    GPIOA->AFR[0]  = (GPIOA->AFR[0]  & ~(0xFFu << 8u)) | (0x77u << 8u); /* AF7 */

    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    (void)RCC->APB1ENR1;                          /* read-back: clock enable fence */

    USART2->CR1 = 0u;                             /* reset */
    USART2->BRR = 64000000u / 115200u;            /* 556 → 115107 baud @ 64 MHz */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;  /* TX+RX, enable */
}

static void USART2_Write(const char *buf, uint16_t len)
{
    for (uint16_t i = 0u; i < len; i++)
    {
        while (!(USART2->ISR & USART_ISR_TXE_Msk));
        USART2->TDR = (uint8_t)buf[i];
    }
    while (!(USART2->ISR & USART_ISR_TC_Msk));  /* wait until last byte is out */
}

#ifdef TEST_MODE
/* Gera PWM 50% duty em TIM16 CH1 (PA6, AF1) para validar o pipeline V→F→T.
 * Jumper PA6 → pino de entrada do segmento que quer validar (ex: PA0 = seg 0). */
static void test_pwm_start(uint32_t freq_hz)
{
    __HAL_RCC_TIM16_CLK_ENABLE();

    /* PA6 → TIM16_CH1 (AF1) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_6;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM16;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* ARR: freq = 1 MHz / (ARR + 1)  →  ARR = (1 MHz / freq) - 1 */
    uint32_t arr = (1000000UL / freq_hz) - 1UL;

    htim16_test.Instance               = TIM16;
    htim16_test.Init.Prescaler         = 64U - 1U;   /* 64 MHz / 64 = 1 MHz */
    htim16_test.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim16_test.Init.Period            = arr;
    htim16_test.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim16_test.Init.RepetitionCounter = 0U;
    htim16_test.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim16_test) != HAL_OK)
        Error_Handler();

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = (arr + 1U) / 2U;   /* 50% duty cycle */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim16_test, &oc, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    HAL_TIM_PWM_Start(&htim16_test, TIM_CHANNEL_1);
}

/* Atualiza a frequência gerada em tempo real, sem reiniciar o timer.
 * Chamado a cada iteração do loop quando test_freq_hz mudar via Live Expressions. */
static void test_pwm_update(uint32_t freq_hz)
{
    if (freq_hz == 0U) return;
    uint32_t arr = (1000000UL / freq_hz) - 1UL;
    __HAL_TIM_SET_AUTORELOAD(&htim16_test, arr);
    __HAL_TIM_SET_COMPARE(&htim16_test, TIM_CHANNEL_1, (arr + 1U) / 2U);
}
#endif /* TEST_MODE */

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
  (void)file; (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
