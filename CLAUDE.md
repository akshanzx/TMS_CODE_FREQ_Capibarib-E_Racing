# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**TMS (Temperature Monitoring System)** — bare-metal ARM Cortex-M4F firmware for a Formula SAE Brasil electric vehicle accumulator (lithium-ion battery pack).

### Vehicle & Competition Context

- Competition: Formula SAE Brasil 2026 (FSAE-B)
- Vehicle: Formula SAE Electric
- Battery cells: ENEPAQ VTC6 cylindrical 18650 Li-ion (1×4 physical holder format)
- Pack configuration: **80S1P** total — 5 segments of **16S1P** each (16 cells in series per segment, 80 cells total)
- Segment voltage: ~57.6V nominal / ~67.2V max (well under the 120V FSAE per-segment limit)
- Total pack voltage: ~288V nominal / ~336V max
- Each segment has its own internal temperature sensor bus (already present on the cell-monitoring network)

### The Problem This Solves

FSAE rules require **galvanic isolation** between each high-voltage accumulator segment and the low-voltage (GLV) vehicle systems. Each segment floats at a different potential. Reading temperature sensors directly from a microcontroller would violate isolation.

### The Isolation Strategy

Each segment's temperature bus already averages the temperatures of its cells into a single analog voltage. A custom circuit reads this voltage and converts it to a **frequency signal** (V→F conversion). Frequency (a digital pulse train) can cross an isolation barrier (optocoupler/capacitive isolator) without any DC conductive path — maintaining full galvanic isolation between the HV segment and the GLV microcontroller.

The STM32G474 firmware then reads the 5 resulting frequency signals and reconstructs the temperatures:

```
[Segment 1–5 internal cell temperature average]
  → existing cell sensor bus (inside HV segment, isolated domain)
  → analog voltage output proportional to temperature
  → V-to-F converter circuit (still in HV domain)
  → isolation barrier (optocoupler/digital isolator)
  → frequency pulse train in GLV domain
  → STM32 Timer Input Capture + DMA (this firmware)
  → frequency → voltage lookup table (1800–5450 Hz → 1.30–2.44V)
  → voltage → temperature interpolation (−40°C to +120°C)
  → UART output to vehicle data system
```

### FSAE Accumulator Rules Context (2026)

- The AMS must monitor temperature of at least **30% of cells**, distributed evenly across all segments.
- Each accumulator segment must be **galvanically isolated** from the chassis/GLV system (this firmware's whole reason for existing).
- Segment voltage must remain ≤ 120V per segment; total TS voltage limits apply.
- An IMD (Isolation Monitoring Device) must continuously verify isolation between TS and chassis.
- The frequency-based isolation approach satisfies these rules by ensuring zero conductive path between HV segment and the STM32 reading the data.

## Build

The build system is GNU Make, managed by STM32CubeIDE. Build artifacts are generated in the `Debug/` directory.

```bash
# Build (from repo root or Debug/)
cd Debug && make all

# Clean
cd Debug && make clean
```

**Outputs:** `Debug/TMS_CODE_V2.elf`, `.hex`, `.bin`, `.map`, `.list`

**Toolchain:** `arm-none-eabi-gcc 13.3.rel1` with flags:
- `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb`
- `--specs=nano.specs --specs=nosys.specs`

There is no formal test framework or lint tooling — this is bare-metal firmware validated via UART output and hardware observation.

## Flashing & Debugging

Flash via STM32CubeIDE using the ST-Link probe and the included launch configuration (`TMS_CODE_V2 Debug.launch`). Command-line flashing requires OpenOCD + GDB.

## Architecture

### Data Flow

```
Rising edges on 5 GPIO pins (PA0, PA1, PB15, PA11, PA8)
  → TIM1/TIM2/TIM15 Input Capture
  → DMA transfers capture timestamps
  → HAL_TIM_IC_CaptureCallback() sets ready flag
  → Main loop: frequency = 1_000_000 / timestamp_diff_µs
  → Lookup table: frequency → voltage (1800–5450 Hz → 1.30–2.44V)
  → Interpolation: voltage → temperature (−40°C to +120°C)
  → UART output (115200 baud, 8N1)
```

### Key Files

| File | Role |
|------|------|
| `Core/Src/main.c` | All application logic: init, main loop, ISR callbacks |
| `Core/Inc/main.h` | GPIO pin definitions and application constants |
| `TMS_CODE_V2.ioc` | STM32CubeMX peripheral configuration (source of truth for HW config) |
| `STM32G474RETX_FLASH.ld` | Linker script (512KB Flash @ 0x8000000, 128KB RAM @ 0x20000000) |
| `Core/Inc/stm32g4xx_hal_conf.h` | Selects which HAL modules are compiled in |
| `Drivers/STM32G4xx_HAL_Driver/` | ST-provided HAL — do not edit manually |

### Pin Mapping

| Pin | Timer Channel | Function |
|-----|--------------|---------|
| PA0 | TIM2 CH1 | Connection 1 |
| PA1 | TIM2 CH2 | Connection 2 |
| PB15 | TIM15 CH2 | Connection 3 |
| PA11 | TIM1 CH4 | Connection 4 |
| PA8 | TIM1 CH1 | Connection 5 |

### Timer Configuration

Prescaler = 63 → 1 MHz timer clock (1 µs resolution) from 64 MHz system clock. Frequency is computed as `1_000_000 / (capture2 - capture1)` in the ISR callback.

## STM32CubeIDE Workflow

1. Edit peripheral configuration in `TMS_CODE_V2.ioc` using the CubeMX visual editor.
2. CubeMX regenerates `MX_*_Init()` functions in `main.c`.
3. **User code must stay inside `/* USER CODE BEGIN */` / `/* USER CODE END */` blocks** — code outside these markers is overwritten on regeneration.
4. Build and flash via STM32CubeIDE or command-line make.

## Enabled HAL Modules

TIM, UART, GPIO, DMA, RCC, FLASH, PWR, CORTEX, EXTI. ADC, CAN, I2C, SPI, DAC, and RTC are disabled — do not enable them without updating `stm32g4xx_hal_conf.h` and the `.ioc` file.
