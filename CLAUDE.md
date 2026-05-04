# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**TMS (Temperature Monitoring System)** — bare-metal ARM Cortex-M4F firmware for a Formula SAE Brasil electric vehicle accumulator (lithium-ion battery pack).

### Vehicle & Competition Context

- Competition: Formula SAE Brasil 2026 (FSAE-B)
- Battery cells: ENEPAQ VTC6 cylindrical 18650 Li-ion
- Pack configuration: **80S1P** total — 5 segments of **16S1P** each (16 cells in series per segment)
- Segment voltage: ~57.6V nominal / ~67.2V max (≤ 120V FSAE per-segment limit)
- Total pack voltage: ~288V nominal / ~336V max

### The Isolation Strategy

FSAE rules require galvanic isolation between each HV accumulator segment and the GLV vehicle system. Each segment's 16 NTC sensors are read via a 16-channel analog MUX. The selected channel's voltage drives a V-to-F converter; the resulting pulse train crosses an optical isolator (no DC path). The STM32G474 reads 5 isolated pulse trains via Timer Input Capture + DMA.

```
16 NTC sensors (per segment)
  → 16-ch analog MUX (S0–S3 address bus: PB4–PB7, shared across all 5 segments)
  → V-to-F converter (HV domain)
  → optical isolator
  → frequency pulse train (GLV domain)
  → STM32 Timer Input Capture + DMA
  → freq → voltage (1800–5450 Hz → 1.30–2.44V)
  → voltage → temperature (−40 °C to +120 °C, lookup table + EMA filter)
  → CAN (Orion 2 BMS + summary frame) + UART
```

**Scan cycle (~320 ms):** for each of 16 MUX channels → drive address bus → wait `MUX_SETTLE_MS` (10 ms) → arm DMA on all 5 timers simultaneously → wait `MUX_MEASURE_MS` (2 ms) → compute freq/voltage/temp for each segment.

**`NUM_SEGMENTS` is currently `1`** in `main.c` — single-segment mode for bring-up. Change to `5` for full-pack operation.

## Build

```bash
cd Debug && make all   # build
cd Debug && make clean # clean
```

**Toolchain:** `arm-none-eabi-gcc 13.3.rel1`
**Flags:** `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb --specs=nano.specs --specs=nosys.specs`
**Outputs:** `Debug/TMS_CODE_V2.elf`, `.hex`, `.bin`, `.map`, `.list`

Flash via STM32CubeIDE + ST-Link (`TMS_CODE_V2 Debug.launch`), or OpenOCD + GDB.

There is no formal test framework — validated via UART output, CAN monitor, and hardware observation.

## Architecture

### Key Files

| File | Role |
|------|------|
| `Core/Src/main.c` | All application logic: MUX scan loop, CAN/UART output, TEST_MODE, temperature conversion |
| `Core/Inc/main.h` | MUX address pin and CAN GPIO pin defines |
| `Core/Src/stm32g4xx_hal_msp.c` | DMA config for TIM1/2/3 and FDCAN1 GPIO init (PB8/PB9, AF9) |
| `TMS_CODE_V2.ioc` | STM32CubeMX config (source of truth for TIM1/2/3, DMA, GPIO; FDCAN and USART2 are manually coded, not generated) |
| `STM32G474RETX_FLASH.ld` | Linker script (512KB Flash @ 0x8000000, 128KB RAM @ 0x20000000) |
| `Core/Inc/stm32g4xx_hal_conf.h` | Selects which HAL modules are compiled in |
| `.claude/dash/dashboard.py` | Python/tkinter PC dashboard — reads UART, displays Segment 0 live |

### Pin Mapping

| Pin | Peripheral | Function |
|-----|-----------|---------|
| PA0 | TIM2 CH1 (AF1) | Segment 0 V-to-F input |
| PA1 | TIM2 CH2 (AF1) | Segment 1 V-to-F input |
| PB1 | TIM3 CH4 (AF2) | Segment 2 V-to-F input |
| PB11 | TIM2 CH4 (AF1) | Segment 3 V-to-F input |
| PA8 | TIM1 CH1 (AF6) | Segment 4 V-to-F input |
| PB4–PB7 | GPIO_OUT | MUX_S0–S3 address bus (LSB→MSB) |
| PA2 | USART2_TX (AF7) | ST-Link VCP → PC dashboard (115200 baud) |
| PA3 | USART2_RX (AF7) | ST-Link VCP (wired but unused) |
| PB8 | FDCAN1_RX (AF9) | CAN bus receive |
| PB9 | FDCAN1_TX (AF9) | CAN bus transmit |
| PA6 | TIM16 CH1 (AF1) | TEST_MODE only: PWM frequency generator |

### Timer Configuration

TIM1, TIM2, TIM3 input-capture timers: **Prescaler = 64−1** → 64 MHz ÷ 64 = **1 MHz** (1 µs resolution), ARR = 65535. DMA captures exactly 2 rising edges per channel. Frequency = `1_000_000 / (cap[1] − cap[0])` with 16-bit wrap-around correction.

### CAN Output (FDCAN1, 500 kbps classic CAN)

- **10 extended-ID frames to Orion 2 BMS** (IDs `0x1839F380`–`0x1839F389`): 8 temperatures per frame, encoding `data[byte] = (uint8_t)(temp_°C + 40)`. Msg 0 = Seg 0 cells 0–7, Msg 1 = Seg 0 cells 8–15, … Msg 9 = Seg 4 cells 8–15.
- **1 standard-ID summary frame** (ID `0x100`) to secondary MCU: T_min/avg/max + cell/segment indices of hottest and coldest cell.
- Transmit-only; all incoming frames rejected via global filter.

### UART Output (USART2, 115200 8N1)

One line per cell of Segment 0 after each full scan:
```
S<seg>,C<cell>,<freq_Hz>,<volt_V>,<temp_C>\r\n
# Example: S0,C03,2480.50,1.840,36.12
```
USART2 is register-level (not via HAL_UART). MUX address lines were moved from PA2–PA5 to PB4–PB7 to free PA2/PA3 for USART2 — hardware must match.

### TEST_MODE

`#define TEST_MODE` (top of `main.c`) enables TIM16 CH1 (PA6) as a 50%-duty PWM generator. Jumper PA6 to any segment input pin (e.g., PA0 = Seg 0) to test the full conversion pipeline without real HV hardware. `volatile uint32_t test_freq_hz` (default 4000 Hz, valid range 1800–5450) can be changed live via STM32CubeIDE **Live Expressions**; the main loop applies the change each iteration. Remove `#define TEST_MODE` for production builds.

### PC Dashboard

```bash
cd .claude/dash
pip install -r requirements.txt
python dashboard.py
```

Connects to the ST-Link VCP COM port, parses UART lines, and shows Segment 0's 16 cells with color-coded temperature: green < 45 °C, yellow 45–59 °C, red ≥ 60 °C.

## STM32CubeIDE Workflow

1. Edit peripherals in `TMS_CODE_V2.ioc` (CubeMX visual editor).
2. CubeMX regenerates `MX_*_Init()` — **user code must stay inside `/* USER CODE BEGIN */` / `/* USER CODE END */` blocks**.
3. **FDCAN and USART2 are not in the `.ioc`** — initialized manually in `FDCAN_Config()` / `USART2_Init()` inside `USER CODE` blocks. `HAL_FDCAN_MspInit()` in `stm32g4xx_hal_msp.c` configures the CAN GPIO. Do not add them to CubeMX without merging carefully.
4. MUX GPIO (PB4–PB7) is also manually initialized inside a `USER CODE` block in `MX_GPIO_Init()`.

## Enabled HAL Modules

TIM, DMA, GPIO, RCC, FLASH, PWR, CORTEX, EXTI, FDCAN. ADC, HAL_UART (classic UART HAL), I2C, SPI, DAC, and RTC are disabled — enable only by updating `stm32g4xx_hal_conf.h` and the `.ioc`.
