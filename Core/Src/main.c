/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : FreeRTOS Multi-Sensor Monitor — Main application entry
  *
  * Target:   STM32F429ZIT6 (STM32F429I-DISC1 Discovery board)
  * Clock:    180 MHz (HSE → PLL)
  * UART:     USART1 @ 115200 baud (PA9=TX, PA10=RX)
  * SPI:      SPI5 Mode 0 (PF7=SCK, PF8=MISO, PF9=MOSI, PC1=CS)
  * ADC:      ADC1 Channel 18 (internal temperature sensor)
  * Button:   PA0 (EXTI0, rising edge)
  * LEDs:     PG13 (green), PG14 (red)
  *
  * System Architecture:
  *
  *   ┌─────────────┐    sensorQueue    ┌──────────────┐
  *   │ Sensor Task ├───────────────────► Display Task │──► UART
  *   │ (AboveNorm) │   (10 packets)    │   (Normal)   │
  *   └─────────────┘                   └──────┬───────┘
  *                                            │ ▲
  *   ┌─────────────┐    eventQueue     ┌──────┘ │
  *   │ Event Task  ├──────────────────►│        │
  *   │   (High)    │   (5 packets)     └────────┘
  *   └──────▲──────┘
  *          │ buttonSem (binary)
  *   ┌──────┴──────┐
  *   │  EXTI0 ISR  │ ← PA0 button press
  *   └─────────────┘
  *
  * Startup Sequence:
  *   1. HAL_Init() — configures Flash, SysTick, NVIC priority grouping
  *   2. SystemClock_Config() — HSE → PLL → 180 MHz SYSCLK
  *   3. Peripheral init (GPIO, UART, SPI, ADC) — all HAL_Init calls
  *   4. L3GD20 init + verify — before scheduler (uses HAL_Delay)
  *   5. Create FreeRTOS objects — queues, semaphores, mutex
  *   6. Create tasks — sensor, display, event
  *   7. Start scheduler — NEVER RETURNS
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "l3gd20.h"
#include "sensor_task.h"
#include "display_task.h"
#include "event_task.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/

/**
 * @brief HAL peripheral handles.
 *
 * These are the "state machines" that HAL uses to track peripheral status.
 * They must persist for the entire program lifetime (hence file-scope).
 */
SPI_HandleTypeDef  hspi5;
UART_HandleTypeDef huart1;
ADC_HandleTypeDef  hadc1;

/**
 * @brief FreeRTOS synchronization objects.
 *
 * Created in main() before the scheduler starts, used by tasks and ISRs.
 * Declared extern in main.h so all modules can access them.
 *
 * Why create before scheduler?
 *   - FreeRTOS object creation uses pvPortMalloc internally
 *   - Creating after scheduler start is allowed but riskier
 *   - Pre-creation guarantees all objects exist before any task runs
 */
QueueHandle_t     sensorQueue = NULL;
QueueHandle_t     eventQueue  = NULL;
SemaphoreHandle_t buttonSem   = NULL;
SemaphoreHandle_t uartMutex   = NULL;

/* Private function prototypes -----------------------------------------------*/
static void UART_Print(const char *str);

/* Printf redirect -----------------------------------------------------------*/

/**
 * @brief  Redirect printf/puts to UART1.
 *
 * This overrides the syscall _write() which is called by printf/puts/putchar.
 * Only used for pre-scheduler boot messages (before FreeRTOS starts).
 * Once tasks are running, use HAL_UART_Transmit with uartMutex instead.
 *
 * @note   For GCC (arm-none-eabi-gcc) toolchain. IAR and Keil use different
 *         mechanisms (__io_putchar for STM32CubeIDE, fputc for Keil).
 */
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}

/* Private helper functions --------------------------------------------------*/

/**
 * @brief  Print a string via UART (for pre-scheduler boot messages).
 *
 * Simple wrapper around HAL_UART_Transmit. No mutex protection needed
 * because this is only called from main() before the scheduler starts
 * (single-threaded context).
 *
 * @param  str  Null-terminated string to print
 */
static void UART_Print(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

/* Peripheral Initialization Functions ---------------------------------------*/

/**
 * @brief  System Clock Configuration — 180 MHz from HSE.
 *
 * Clock tree:
 *   HSE (8 MHz crystal on DISC1)
 *     → PLL: /8 × 360 /2 = 180 MHz SYSCLK
 *     → AHB prescaler /1 = 180 MHz HCLK
 *     → APB1 prescaler /4 = 45 MHz (max for APB1)
 *     → APB2 prescaler /2 = 90 MHz (max for APB2)
 *
 * Flash configuration:
 *   - 5 wait states at 180 MHz (per datasheet Table 10)
 *   - Instruction cache, data cache, and prefetch enabled
 *
 * Over-drive mode:
 *   Required to reach 180 MHz on STM32F429 (without it, max is 168 MHz).
 *   Increases internal voltage regulator output.
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Enable Power Control clock and set voltage scaling */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Configure HSE and PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;     /* VCO input: 8 MHz / 8 = 1 MHz */
    RCC_OscInitStruct.PLL.PLLN       = 360;   /* VCO output: 1 MHz × 360 = 360 MHz */
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2; /* SYSCLK: 360 / 2 = 180 MHz */
    RCC_OscInitStruct.PLL.PLLQ       = 7;     /* USB: 360 / 7 ≈ 51.4 MHz (not used) */

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable over-drive mode for 180 MHz operation */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure bus clocks: AHB, APB1, APB2 */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;    /* 180 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;      /* 45 MHz (max) */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;      /* 90 MHz (max) */

    /* 5 wait states for 180 MHz at 3.3V (per datasheet) */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  GPIO Initialization — LEDs, Button, Gyro CS.
 *
 * Pin assignments (from STM32F429I-DISC1 schematic):
 *   PG13: Green LED  — push-pull output, no pull-up/down
 *   PG14: Red LED    — push-pull output, no pull-up/down
 *   PA0:  User button — input with EXTI, rising edge trigger
 *   PC1:  Gyro CS     — push-pull output, start HIGH (deselected)
 *
 * EXTI priority: Set to 6 (above FreeRTOS's configMAX_SYSCALL_INTERRUPT_PRIORITY = 5)
 * so that FreeRTOS API calls from the ISR are safe. If priority were higher
 * (lower number), calling xSemaphoreGiveFromISR would corrupt FreeRTOS state.
 */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clocks for all used ports */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* ---- LEDs: PG13 (Green), PG14 (Red) ---- */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  /* LEDs don't need fast switching */
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* ---- User Button: PA0 with EXTI rising edge ---- */
    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  /* Interrupt on press (rising edge) */
    GPIO_InitStruct.Pull = GPIO_NOPULL;           /* External pull-down on DISC1 board */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Enable EXTI0 interrupt in NVIC */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    /* ---- Gyroscope CS: PC1 — start HIGH (deselected) ---- */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  /* CS needs fast transitions */
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
 * @brief  USART1 Initialization — 115200 baud, 8N1.
 *
 * Pin mapping:
 *   PA9:  USART1_TX (AF7) — connect to USB-TTL RXD
 *   PA10: USART1_RX (AF7) — connect to USB-TTL TXD
 *
 * Note: The STM32F429I-DISC1 does NOT have a built-in USB-to-serial
 * converter on USART1. You need an external USB-TTL adapter connected
 * to PA9/PA10 (or use the ST-Link VCP on USART3 — but we're using USART1
 * per the project specification).
 *
 * Baud rate calculation:
 *   USART1 is on APB2 (90 MHz).
 *   BRR = fCLK / (16 × baud) = 90000000 / (16 × 115200) ≈ 48.828
 *   HAL handles this calculation internally.
 */
void MX_USART1_UART_Init(void)
{
    /* Enable USART1 and GPIOA clocks */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure PA9 (TX) and PA10 (RX) as USART1 alternate function */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;      /* Pull-up prevents floating RX line */
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure USART1: 115200 baud, 8 data bits, 1 stop bit, no parity */
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  SPI5 Initialization — Mode 0, 8-bit, for L3GD20 gyroscope.
 *
 * Pin mapping (fixed on DISC1 board — L3GD20 is soldered):
 *   PF7: SPI5_SCK  (AF5)
 *   PF8: SPI5_MISO (AF5)
 *   PF9: SPI5_MOSI (AF5)
 *   PC1: CS (GPIO, software controlled)
 *
 * SPI configuration:
 *   - Mode 0 (CPOL=0, CPHA=0): clock idle low, sample on rising edge
 *   - 8-bit data size
 *   - MSB first (standard for most SPI devices)
 *   - Software NSS management (we control CS via GPIO)
 *   - Master mode
 *   - Prescaler: 16 → SPI clock = APB2/16 = 90/16 = 5.625 MHz
 *     (L3GD20 max SPI clock is 10 MHz, so 5.625 MHz is safe)
 */
void MX_SPI5_Init(void)
{
    /* Enable SPI5 and GPIOF clocks */
    __HAL_RCC_SPI5_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* Configure PF7 (SCK), PF8 (MISO), PF9 (MOSI) as SPI5 alternate function */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* Configure SPI5 */
    hspi5.Instance               = SPI5;
    hspi5.Init.Mode              = SPI_MODE_MASTER;
    hspi5.Init.Direction         = SPI_DIRECTION_2LINES;  /* Full-duplex */
    hspi5.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi5.Init.CLKPolarity       = SPI_POLARITY_LOW;      /* CPOL = 0 */
    hspi5.Init.CLKPhase          = SPI_PHASE_1EDGE;       /* CPHA = 0 */
    hspi5.Init.NSS               = SPI_NSS_SOFT;          /* Software CS */
    hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /* 5.625 MHz */
    hspi5.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi5.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi5.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi5.Init.CRCPolynomial     = 10;  /* Not used, default value */

    if (HAL_SPI_Init(&hspi5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  ADC1 Initialization — Internal temperature sensor (Channel 18).
 *
 * The STM32F4 internal temperature sensor is connected to ADC1_IN18.
 * It provides a rough (~±1.5°C) temperature reading of the die.
 *
 * Configuration:
 *   - 12-bit resolution (0-4095)
 *   - Single conversion mode (we trigger each read manually)
 *   - Sample time: 480 cycles (recommended for temp sensor stability)
 *   - No DMA, no interrupts — simple polling
 *
 * The internal temp sensor requires enabling TSVREFE bit in ADC_CCR,
 * which HAL handles when we configure Channel 18.
 */
void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /* Enable ADC1 clock */
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* Configure ADC1 */
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;  /* 90/4 = 22.5 MHz */
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;  /* Single shot — we trigger manually */
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * Configure ADC channel 18 (internal temperature sensor).
     *
     * Sample time of 480 cycles is required by the datasheet for the
     * internal temperature sensor. Shorter sample times give noisy readings.
     * At 22.5 MHz ADC clock: 480 cycles = 21.3 µs per sample.
     */
    sConfig.Channel      = ADC_CHANNEL_18;         /* Internal temp sensor */
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable the internal temperature sensor and VREFINT */
    ADC->CCR |= ADC_CCR_TSVREFE;
}

/* Main Application ----------------------------------------------------------*/

/**
 * @brief  Application entry point.
 *
 * This function initializes all hardware, creates FreeRTOS objects,
 * and starts the scheduler. It should NEVER return.
 *
 * If vTaskStartScheduler() returns, it means the idle task or timer task
 * could not be created (out of heap). The while(1) at the end catches this.
 */
int main(void)
{
    char buf[80];

    /*
     * Step 1: HAL initialization.
     * This configures:
     *   - Flash interface (prefetch, instruction cache, data cache)
     *   - SysTick timer (1ms tick for HAL_Delay)
     *   - NVIC priority grouping (4 bits preemption, 0 bits subpriority)
     */
    HAL_Init();

    /*
     * Step 2: System clock configuration.
     * HSE (8 MHz) → PLL → 180 MHz SYSCLK
     * Must be done early — all peripheral clocks derive from SYSCLK.
     */
    SystemClock_Config();

    /*
     * Step 3: Peripheral initialization.
     * Order matters: GPIO first (enables clocks for ports), then peripherals.
     */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_SPI5_Init();
    MX_ADC1_Init();

    /* ---- Boot Banner ---- */
    UART_Print("\r\n");
    UART_Print("========================================\r\n");
    UART_Print("  FreeRTOS Multi-Sensor Monitor v1.0\r\n");
    UART_Print("  STM32F429I-DISC1 | 180 MHz\r\n");
    UART_Print("========================================\r\n");

    snprintf(buf, sizeof(buf), "SystemCoreClock: %lu Hz\r\n", SystemCoreClock);
    UART_Print(buf);

    /*
     * Step 4: Initialize and verify the L3GD20 gyroscope.
     *
     * This MUST happen before the scheduler starts because L3GD20_Init()
     * uses HAL_Delay(), which relies on SysTick running normally.
     * Once FreeRTOS starts, SysTick is repurposed for the scheduler tick.
     */
    L3GD20_Init();
    uint8_t gyro_id = L3GD20_ReadID();

    snprintf(buf, sizeof(buf), "L3GD20 WHO_AM_I: 0x%02X %s\r\n",
             gyro_id,
             (gyro_id == 0xD4 || gyro_id == 0xD7) ? "(OK)" : "(FAIL)");
    UART_Print(buf);

    /*
     * Step 5: Create FreeRTOS synchronization objects.
     *
     * Creating all objects before any tasks ensures no task runs
     * with a NULL handle. If creation fails (out of heap), we catch
     * it here before the system becomes multi-threaded.
     */

    /* Sensor data queue: depth 10 allows ~1 second of buffering at 100ms rate */
    sensorQueue = xQueueCreate(10, sizeof(SensorData_t));

    /* Event queue: depth 5 is plenty — button presses are rare */
    eventQueue = xQueueCreate(5, sizeof(EventData_t));

    /*
     * Binary semaphore for ISR → EventTask signaling.
     * Starts "empty" (taken), so EventTask blocks until first ISR fires.
     * Binary (not counting) because we only need "at least one event happened."
     */
    buttonSem = xSemaphoreCreateBinary();

    /*
     * Mutex for UART access.
     * Unlike a binary semaphore, a mutex has:
     *   - Priority inheritance (prevents priority inversion)
     *   - Ownership tracking (only the taker can give it)
     * Both are essential for a shared resource like UART.
     */
    uartMutex = xSemaphoreCreateMutex();

    /* Verify all objects were created successfully */
    if (sensorQueue == NULL || eventQueue == NULL ||
        buttonSem == NULL || uartMutex == NULL)
    {
        UART_Print("ERROR: Failed to create FreeRTOS objects!\r\n");
        Error_Handler();
    }

    UART_Print("FreeRTOS objects created successfully\r\n");

    /*
     * Step 6: Create tasks.
     *
     * Task priorities (higher number = higher priority):
     *   osPriorityHigh        (40) — EventTask: fast button response
     *   osPriorityAboveNormal (32) — SensorTask: consistent sampling
     *   osPriorityNormal      (24) — DisplayTask: best-effort display
     *
     * Stack sizes:
     *   512 words (2048 bytes) — Sensor and Display tasks
     *     (need space for snprintf buffers, SPI buffers, and local vars)
     *   256 words (1024 bytes) — Event task
     *     (minimal stack — no complex formatting)
     *
     * Note: osPriorityXxx values are from CMSIS-RTOS2 mapping.
     * They map to FreeRTOS priorities internally.
     */
    BaseType_t ret;

    ret = xTaskCreate(SensorTask, "Sensor", 512, NULL, osPriorityAboveNormal, NULL);
    if (ret != pdPASS)
    {
        UART_Print("ERROR: Failed to create SensorTask!\r\n");
        Error_Handler();
    }

    ret = xTaskCreate(DisplayTask, "Display", 512, NULL, osPriorityNormal, NULL);
    if (ret != pdPASS)
    {
        UART_Print("ERROR: Failed to create DisplayTask!\r\n");
        Error_Handler();
    }

    ret = xTaskCreate(EventTask, "Event", 256, NULL, osPriorityHigh, NULL);
    if (ret != pdPASS)
    {
        UART_Print("ERROR: Failed to create EventTask!\r\n");
        Error_Handler();
    }

    UART_Print("All tasks created. Starting scheduler...\r\n\r\n");

    /*
     * Step 7: Start the FreeRTOS scheduler.
     *
     * THIS CALL NEVER RETURNS (in normal operation).
     * After this call:
     *   - SysTick is reconfigured for FreeRTOS tick (1ms)
     *   - Context switching begins
     *   - HAL_Delay() NO LONGER WORKS (use vTaskDelay instead)
     *   - The idle task handles time-critical HAL tick increments
     *
     * If it returns, it means the idle task couldn't be created.
     */
    vTaskStartScheduler();

    /*
     * If we reach here, something went very wrong.
     * Possible causes:
     *   - Not enough heap for the idle task
     *   - configTOTAL_HEAP_SIZE too small in FreeRTOSConfig.h
     */
    UART_Print("ERROR: Scheduler failed to start!\r\n");

    while (1)
    {
        /* Should never reach here */
    }
}

/**
 * @brief  Error handler — called on unrecoverable peripheral init failures.
 *
 * In a production system, you might:
 *   - Log the error to flash
 *   - Trigger a watchdog reset
 *   - Enter a safe state
 *
 * For this demo, we just blink both LEDs rapidly as an error indicator.
 */
void Error_Handler(void)
{
    __disable_irq();

    /* Both LEDs on as error indicator */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_SET);

    while (1)
    {
        /* Intentionally empty */
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  HAL assert failed callback.
 *
 * Called by HAL functions when an invalid parameter is detected
 * (only when USE_FULL_ASSERT is defined — disabled in Release builds).
 *
 * @param  file  Source file where the assertion failed
 * @param  line  Line number of the failed assertion
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;

    /* In debug: set a breakpoint here to catch the assertion */
    Error_Handler();
}
#endif
