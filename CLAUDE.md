# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**TMS (Temperature Monitoring System)** — bare-metal ARM Cortex-M4F firmware for a Formula SAE Brasil electric vehicle accumulator (lithium-ion battery pack).

### Vehicle & Competition Context

- Competition: Formula SAE Brasil 2026 (FSAE-B)
- Battery cells: ENEPAQ VTC6 cylindrical 18650 Li-ion
- Pack configuration: **80S1P** — 5 segments of **16S1P** each
- Segment voltage: ~57.6V nominal / ~67.2V max
- Total pack voltage: ~288V nominal / ~336V max

### The Isolation Problem

FSAE rules require galvanic isolation between each HV accumulator segment and the low-voltage (GLV) system. The solution: each segment's 16 NTC cell sensors are muxed to a V→F converter. The frequency signal crosses an optocoupler without any DC path, then the STM32 reads it via Timer Input Capture + DMA.

## Build

Build system: GNU Make via STM32CubeIDE. Artifacts go in `Debug/`.

```bash
cd Debug && make all    # build
cd Debug && make clean  # clean
```

**Outputs:** `Debug/TMS_V1.elf`, `.hex`, `.bin`, `.map`, `.list`

**Toolchain:** `arm-none-eabi-gcc 13.3.rel1`  
Flags: `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb --specs=nano.specs --specs=nosys.specs`

No test framework or lint tooling — validated via UART output and hardware observation.

## Flashing & Debugging

Flash via STM32CubeIDE with ST-Link probe. Command-line: OpenOCD + GDB.

## Architecture

### Data Flow (per scan cycle, ~320 ms for 16 channels)

```
for ch = 0..15:
  Drive MUX_S0–S3 (PB4–PB7) → selects cell ch on ALL 5 segments simultaneously
  Wait MUX_SETTLE_MS (10 ms) for V-to-F output to stabilise
  Start DMA Input Capture on 5 timer channels
  Wait MUX_MEASURE_MS (2 ms) — captures 2 rising edges per channel
  compute_frequency() → EMA filter → freq_to_voltage() → voltage_to_temp()

After full 16-channel scan:
  ComputeStats() → t_min, t_avg, t_max with open-wire exclusion
  CAN_SendOrion2Data()  — 10 × 8-byte extended-ID frames to Orion 2 BMS
  CAN_SendSummaryPacket() — 1 × 8-byte standard-ID frame to secondary MCU
  USART2_Write() — one line per cell of segment 0 (debug, 115200 baud)
```

### Key Implementation Details

- **Frequency range:** 1829–5515 Hz → 1.30–2.44 V → −40°C to +120°C
- **EMA filter:** α = 0.3 applied to raw frequency before conversion
- **Open-wire detection:** `freq > FREQ_OPEN_WIRE_HZ (7000 Hz)` → immediate −999; `freq == 0` for `OPEN_WIRE_MISS_MAX (3)` consecutive scans → −999. Last valid reading preserved on recovery.
- **Array sizing:** All segment arrays are `[MAX_SEGMENTS=5]` regardless of `NUM_SEGMENTS`. This is intentional — `process_segment()` is called for all 5 hardware channels unconditionally; using `[NUM_SEGMENTS]` caused a buffer overflow that corrupted `t_min`.
- **TEST_MODE** (`#define TEST_MODE` in `main.c`): enables a TIM16 PWM generator on PA6 (AF1) producing a 50%-duty square wave. Jumper PA6 → segment input pin to validate the full pipeline without hardware. Change `test_freq_hz` via Live Expressions at runtime. Channel 14 of segment 0 is skipped in `ComputeStats()` while TEST_MODE is active.

### CAN Output (FDCAN1, 500 kbps, PB8 RX / PB9 TX)

**Orion 2 BMS** — 10 extended-ID frames (29-bit), base `0x1839F380`:
- Msg 0–1: Seg 0 cells 0–7, 8–15; Msg 2–3: Seg 1 … Msg 8–9: Seg 4
- Encoding: `data_byte = (uint8_t)(temp_°C + 40)`

**Summary frame** — standard ID `0x100`:
- `[T_min+40, min_cell, min_seg, T_avg+40, T_max+40, max_cell, max_seg, 0x00]`

### Pin Mapping

| Pin | Peripheral | Function |
|-----|-----------|---------|
| PA0 | TIM2 CH1 | Segment 0 V-to-F input |
| PA1 | TIM2 CH2 | Segment 1 V-to-F input |
| PB1 | TIM3 CH4 | Segment 2 V-to-F input |
| PB11 | TIM2 CH4 | Segment 3 V-to-F input |
| PA8 | TIM1 CH1 | Segment 4 V-to-F input |
| PA2 | USART2 TX (AF7) | ST-Link VCP → PC |
| PA6 | TIM16 CH1 (AF1) | TEST_MODE PWM output |
| PB4–PB7 | GPIO_OUT | MUX S0–S3 address bus |
| PB8–PB9 | FDCAN1 RX/TX (AF9) | CAN bus to Orion 2 BMS |

### Timer Configuration

Prescaler = 64−1 → 1 MHz timer clock (1 µs resolution) from 64 MHz HSI-PLL clock.  
Frequency = `1_000_000 / (cap[1] − cap[0])` with 16-bit wrap-around handling (`TIMER_OVERFLOW = 65536`).

### Key Files

| File | Role |
|------|------|
| `Core/Src/main.c` | All application logic: init, scan loop, CAN/UART output, TEST_MODE |
| `Core/Inc/main.h` | MUX and CAN pin defines |
| `TMS_V1.ioc` | STM32CubeMX peripheral config (source of truth for HW) |
| `Core/Src/stm32g4xx_hal_msp.c` | MSP callbacks: FDCAN1 clock + GPIO, DMA bindings |
| `STM32G474RETX_FLASH.ld` | Linker script (512 KB Flash, 128 KB RAM) |
| `Core/Inc/stm32g4xx_hal_conf.h` | Selects HAL modules compiled in |

## STM32CubeIDE Workflow

1. Edit peripherals in `TMS_V1.ioc` using the CubeMX visual editor.
2. CubeMX regenerates `MX_*_Init()` functions in `main.c`.
3. **All user code must stay inside `/* USER CODE BEGIN */` / `/* USER CODE END */` blocks** — anything outside is overwritten on regeneration.
4. `FDCAN_Config()`, `USART2_Init()`, and all GPIO for MUX/CAN pins are in user-code blocks and survive regeneration.

## Enabled HAL Modules

TIM, FDCAN, GPIO, DMA, RCC, FLASH, PWR, CORTEX, EXTI.  
ADC, I2C, SPI, DAC, RTC are disabled — do not enable without updating `stm32g4xx_hal_conf.h` and the `.ioc`.

## Bring-up vs. Full-pack

`NUM_SEGMENTS` is currently `1` (single-segment bring-up). Change to `5` for full-pack operation. `MAX_SEGMENTS` must remain `5` — never reduce it.
