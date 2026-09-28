# FreeRTOS Multi-Sensor Monitor for STM32F429I-DISC1

A production-quality multi-sensor monitoring system built on FreeRTOS, demonstrating real-time embedded systems concepts including task scheduling, inter-task communication, interrupt handling, and sensor integration.

## Hardware Target

| Component | Details |
|-----------|---------|
| **Board** | STM32F429I-DISC1 (Discovery) |
| **MCU** | STM32F429ZIT6 — ARM Cortex-M4F @ 180 MHz |
| **Gyroscope** | L3GD20 (onboard, SPI5) |
| **Temperature** | Internal ADC temperature sensor |
| **UART** | USART1 @ 115200 baud (PA9=TX, PA10=RX) |
| **LEDs** | PG13 (Green), PG14 (Red) |
| **Button** | PA0 (Blue user button) |

## System Architecture

```
┌─────────────────┐   sensorQueue (10)   ┌───────────────┐
│   Sensor Task   ├─────────────────────►│  Display Task  │──► UART
│  (AboveNormal)  │                      │   (Normal)     │
│  100ms period   │                      └───────┬────────┘
└─────────────────┘                              │ ▲
                                                 │ │
┌─────────────────┐   eventQueue (5)     ┌───────┘ │
│   Event Task    ├─────────────────────►│         │
│    (High)       │                      └─────────┘
└────────▲────────┘
         │ buttonSem (binary)
┌────────┴────────┐
│   EXTI0 ISR     │ ← PA0 button press
│  (minimal ISR)  │
└─────────────────┘
```

## Task Overview

### Sensor Task (`sensor_task.c`) — Priority: Above Normal
- Reads L3GD20 gyroscope XYZ via SPI every **100ms**
- Reads internal temperature sensor via ADC every **500ms**
- Sends `SensorData_t` packets to display via queue
- Toggles red LED as heartbeat indicator
- Falls back to dummy data if gyroscope is not detected

### Display Task (`display_task.c`) — Priority: Normal
- Blocks on sensor queue, formats data with `snprintf`
- Polls event queue (non-blocking) for button events
- Acquires UART mutex before transmitting
- Never uses `sprintf` — always bounded writes

### Event Task (`event_task.c`) — Priority: High
- Waits on binary semaphore from button ISR
- Debounces button (50ms delay + re-read)
- Toggles green LED on valid press
- Sends `EventData_t` to event queue

## Pin Mapping

| Pin | Function | Notes |
|-----|----------|-------|
| PA0 | User Button (EXTI0) | Active high, rising edge interrupt |
| PA9 | USART1 TX | Connect to USB-TTL RXD |
| PA10 | USART1 RX | Connect to USB-TTL TXD |
| PC1 | L3GD20 CS (SPI) | Active low, software controlled |
| PF7 | SPI5 SCK | L3GD20 clock |
| PF8 | SPI5 MISO | L3GD20 data out |
| PF9 | SPI5 MOSI | L3GD20 data in |
| PG13 | Green LED | Toggle on button press |
| PG14 | Red LED | Heartbeat (sensor task) |

## Building

### Prerequisites
- STM32CubeIDE (or arm-none-eabi-gcc toolchain)
- STM32F4 HAL drivers (STM32Cube_FW_F4)
- FreeRTOS kernel (included in STM32Cube firmware)

### With STM32CubeIDE
1. Create a new STM32 project for STM32F429ZIT6
2. Copy `Core/Src/` and `Core/Inc/` files into the project
3. Ensure FreeRTOS middleware is enabled in CubeMX
4. Build and flash

### With Makefile / arm-none-eabi-gcc
1. Set up the toolchain and link against HAL + FreeRTOS libraries
2. Include the STM32F4 HAL headers and FreeRTOS port for CM4F
3. Compile all `Core/Src/*.c` files
4. Link with startup file (`startup_stm32f429xx.s`) and linker script

## UART Output

Connect a USB-to-TTL serial adapter to PA9 (TX) and PA10 (RX). Open a terminal at **115200 baud, 8N1**:

```
========================================
  FreeRTOS Multi-Sensor Monitor v1.0
  STM32F429I-DISC1 | 180 MHz
========================================
SystemCoreClock: 180000000 Hz
L3GD20 WHO_AM_I: 0xD4 (OK)
FreeRTOS objects created successfully
All tasks created. Starting scheduler...

[SensorTask] Started
[DisplayTask] Started
[EventTask] Started

[  100 ms] Gyro: X=   -12 Y=    45 Z=  -128 | Temp: 27.3 C

[  200 ms] Gyro: X=   -10 Y=    42 Z=  -130 | Temp: 27.3 C

[EVENT] Button ISR -> Semaphore -> Task
[EVENT] Button pressed at 350 ms
```

## Debug Bug Scenarios

Three intentional bug scenarios are included for learning:

### BUG 1 — Stack Overflow
```c
// Define BUG_STACK_OVERFLOW to enable
// DisplayTask allocates 1024-byte buffer on 2048-byte stack → BOOM
```
**Symptom:** Both LEDs turn on solid, "STACK OVERFLOW" message on UART  
**Fix:** Use bounded buffers, increase stack size, or reduce local variables

### BUG 2 — Missing Volatile
```c
// Define BUG_NO_VOLATILE to enable
// Shared variable without volatile → compiler optimizes away ISR updates
```
**Symptom:** Task never sees counter updates from ISR (with -O2)  
**Fix:** Add `volatile` qualifier to any variable shared between ISR and task

### BUG 3 — Priority Inversion
```c
// Define BUG_PRIORITY_INVERSION to enable
// Binary semaphore used as mutex (no priority inheritance)
```
**Symptom:** High-priority task blocked by medium-priority task  
**Fix:** Use `xSemaphoreCreateMutex()` instead of `xSemaphoreCreateBinary()`

## Key Design Decisions

1. **Deferred Interrupt Processing**: Button ISR only gives a semaphore (~100ns). All processing happens in the Event Task. This keeps ISR latency predictable.

2. **Separate Queues**: Sensor data and events use separate queues. Events don't wait behind buffered sensor data.

3. **UART Mutex**: FreeRTOS mutex (not binary semaphore) prevents priority inversion when multiple tasks share the UART.

4. **vTaskDelayUntil()**: Sensor task uses absolute timing for consistent 100ms sampling, regardless of execution time variation.

5. **snprintf Everywhere**: Defense-in-depth against buffer overflows. No `sprintf` calls in the entire codebase.

6. **Graceful Degradation**: If the L3GD20 fails, the system continues with dummy data instead of crashing.

## Project Structure

```
Core/
├── Inc/
│   ├── main.h                 # Shared types, pin defines, extern declarations
│   ├── FreeRTOSConfig.h       # FreeRTOS kernel configuration
│   ├── stm32f4xx_it.h         # Interrupt handler declarations
│   ├── l3gd20.h               # Gyroscope driver API
│   ├── sensor_task.h          # Sensor task declaration
│   ├── display_task.h         # Display task declaration
│   └── event_task.h           # Event task declaration
└── Src/
    ├── main.c                 # Init, task creation, scheduler start
    ├── l3gd20.c               # L3GD20 SPI driver implementation
    ├── sensor_task.c          # Gyro + temp reading task
    ├── display_task.c         # UART formatted output task
    ├── event_task.c           # Button event handler task
    ├── stm32f4xx_it.c         # Interrupt handlers (EXTI0)
    └── freertos.c             # FreeRTOS hooks (stack overflow, idle)
```

## License

MIT License — see [LICENSE](LICENSE) for details.
