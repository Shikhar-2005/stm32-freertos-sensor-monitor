/**
  ******************************************************************************
  * @file           : stm32f4xx_it.c
  * @brief          : Interrupt service routines (ISR) for STM32F4xx
  *
  * DESIGN PHILOSOPHY FOR ISRs:
  *
  *   ISRs must be FAST and MINIMAL. Every microsecond spent in an ISR is
  *   a microsecond that other interrupts can't fire and tasks can't run.
  *
  *   Rules for ISRs in a FreeRTOS system:
  *   1. Do the absolute minimum work (acknowledge interrupt, read status)
  *   2. Defer processing to a task via semaphore/queue/notification
  *   3. Use only "FromISR" FreeRTOS API variants
  *   4. NEVER use HAL_Delay(), printf(), or any blocking function
  *   5. NEVER toggle LEDs or do "debug" I/O (it adds latency)
  *   6. ALWAYS call portYIELD_FROM_ISR() if a higher-priority task was woken
  *
  *   The portYIELD_FROM_ISR() call is critical: without it, the woken task
  *   doesn't run until the next scheduler tick (up to 1ms later). With it,
  *   the context switch happens immediately upon ISR exit.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "main.h"

/* External variables --------------------------------------------------------*/

/*
 * These are defined in main.c and declared extern in main.h.
 * The ISR needs buttonSem to signal the event task.
 */

/* Cortex-M4 System Interrupt Handlers ---------------------------------------*/

/**
 * @brief  Non-Maskable Interrupt handler.
 *
 * NMI can be triggered by:
 *   - Clock Security System (CSS) detecting HSE failure
 *   - Flash ECC errors (on some STM32 variants)
 *   - Software via SCB->ICSR
 *
 * In production, you'd handle CSS failover to HSI here.
 * For this demo, we just halt.
 */
void NMI_Handler(void)
{
    while (1)
    {
        /* Intentionally empty */
    }
}

/**
 * @brief  Hard Fault handler.
 *
 * Hard Fault occurs when:
 *   - Bus fault escalates (e.g., accessing invalid memory)
 *   - Usage fault escalates (e.g., undefined instruction)
 *   - Any fault during fault handling (double fault)
 *
 * In a debugger, check:
 *   - SCB->CFSR for the specific fault type
 *   - SCB->BFAR / SCB->MMFAR for the faulting address
 *   - Stack frame for the faulting PC (return address)
 */
void HardFault_Handler(void)
{
    while (1)
    {
        /* Intentionally empty */
    }
}

/**
 * @brief  Memory Management Fault handler.
 *
 * Triggered by MPU violations (if MPU is configured) or
 * execution from non-executable regions.
 */
void MemManage_Handler(void)
{
    while (1)
    {
        /* Intentionally empty */
    }
}

/**
 * @brief  Bus Fault handler.
 *
 * Triggered by errors on the AHB bus:
 *   - Accessing unmapped memory regions
 *   - Accessing peripherals with clock disabled
 *   - DMA errors on certain transfers
 */
void BusFault_Handler(void)
{
    while (1)
    {
        /* Intentionally empty */
    }
}

/**
 * @brief  Usage Fault handler.
 *
 * Triggered by:
 *   - Undefined instructions
 *   - Unaligned access (if configured to fault)
 *   - Division by zero (if configured to fault)
 *   - Invalid state (e.g., switching to ARM mode on Cortex-M)
 */
void UsageFault_Handler(void)
{
    while (1)
    {
        /* Intentionally empty */
    }
}

/**
 * @brief  Debug Monitor handler.
 *
 * Used by debug tools for software breakpoints and watchpoints.
 * Left empty — the debugger handles this internally.
 */
void DebugMon_Handler(void)
{
    /* Nothing to do */
}

/*
 * NOTE: SVC_Handler, PendSV_Handler, and SysTick_Handler are NOT defined here.
 *
 * FreeRTOS provides its own implementations:
 *   - SVC_Handler → vPortSVCHandler (used to start the first task)
 *   - PendSV_Handler → xPortPendSVHandler (context switching)
 *   - SysTick_Handler → xPortSysTickHandler (tick increment)
 *
 * These are either:
 *   a) Defined in port.c of the FreeRTOS Cortex-M4F port, or
 *   b) Mapped via #define in FreeRTOSConfig.h
 *
 * Defining them here would cause linker "multiple definition" errors.
 */

/* Peripheral Interrupt Handlers ---------------------------------------------*/

/**
 * @brief  EXTI Line 0 interrupt handler (PA0 — User Button).
 *
 * This is the HARDWARE interrupt entry point. It's called directly by
 * the NVIC when EXTI line 0 triggers. We delegate to HAL's handler,
 * which clears the interrupt flag and calls our callback.
 *
 * Call chain: EXTI0_IRQHandler → HAL_GPIO_EXTI_IRQHandler → HAL_GPIO_EXTI_Callback
 *
 * Why use the HAL wrapper instead of clearing the flag directly?
 *   1. HAL handles edge cases (clearing pending bits correctly)
 *   2. Consistent with the rest of the HAL-based codebase
 *   3. The callback pattern separates HW management from application logic
 */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
 * @brief  EXTI callback for button press — the ONLY application code in ISR context.
 *
 * This callback runs in ISR context (interrupts may be partially disabled,
 * FreeRTOS scheduler is not running). It does exactly ONE thing: signal
 * the event task via a binary semaphore.
 *
 * Why binary semaphore (not queue, not task notification)?
 *   - Semaphore: simplest signaling primitive. "Something happened."
 *   - Queue: overkill. We don't need to pass data from the ISR.
 *   - Task notification: faster, but less portable and harder to debug.
 *
 * The portYIELD_FROM_ISR call:
 *   If giving the semaphore woke up the EventTask (which has higher priority
 *   than whatever was running), we need to context-switch immediately when
 *   this ISR exits. portYIELD_FROM_ISR sets the PendSV bit, which triggers
 *   the context switch after all ISRs have completed.
 *
 * @param  GPIO_Pin  Pin that triggered the interrupt (expected: GPIO_PIN_0)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        /*
         * Give the semaphore from ISR context.
         *
         * xSemaphoreGiveFromISR (not xSemaphoreGive!) because we're in an ISR.
         * The "FromISR" variant:
         *   - Doesn't attempt to block (ISRs can't block)
         *   - Returns whether a higher-priority task was woken
         *   - Is safe to call with interrupts partially disabled
         */
        xSemaphoreGiveFromISR(buttonSem, &xHigherPriorityTaskWoken);

        /*
         * Request context switch if needed.
         *
         * If EventTask (priority High) was blocked on this semaphore,
         * and the currently running task has lower priority, this sets
         * the PendSV flag to trigger a context switch when the ISR exits.
         *
         * If no higher-priority task was woken, this is a no-op.
         */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
