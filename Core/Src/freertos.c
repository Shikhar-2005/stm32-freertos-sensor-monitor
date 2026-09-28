/**
  ******************************************************************************
  * @file           : freertos.c
  * @brief          : FreeRTOS hook functions (callbacks)
  *
  * FreeRTOS calls these "hooks" at specific points in its execution.
  * They must be enabled in FreeRTOSConfig.h:
  *   configUSE_IDLE_HOOK = 1
  *   configCHECK_FOR_STACK_OVERFLOW = 2
  *
  * These hooks are our safety net — they catch catastrophic errors
  * that would otherwise cause silent corruption or hard faults.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Stack overflow hook — called when FreeRTOS detects stack overflow.
 *
 * When configCHECK_FOR_STACK_OVERFLOW is set to 2, FreeRTOS fills each
 * task's stack with a known pattern (0xA5A5A5A5) at creation. On every
 * context switch, it checks if the last 20 bytes of the stack still
 * contain this pattern. If not, the stack has overflowed.
 *
 * Recovery is NOT possible once the stack has overflowed:
 *   - The task's local variables are corrupted
 *   - The return address on the stack may be garbage
 *   - The TCB (Task Control Block) may be corrupted
 *   - Adjacent tasks' stacks may be damaged
 *
 * All we can do is signal the error and halt. The LEDs provide a visual
 * indicator that doesn't require UART (which might be corrupted too).
 *
 * @param  xTask      Handle of the task that overflowed (may be corrupted)
 * @param  pcTaskName Name of the task (may be corrupted)
 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
    /* Suppress unused parameter warnings */
    (void)xTask;
    (void)pcTaskName;

    /*
     * Turn on BOTH LEDs as error indicator.
     * Both LEDs on simultaneously = stack overflow error.
     * (Normal operation: red blinks = sensor task heartbeat,
     *  green toggles = button press)
     */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_SET);

    /*
     * Attempt to print error message.
     * Using a short, fixed string — NOT snprintf, because:
     *   1. The stack is corrupted, so stack-allocated buffers are unsafe
     *   2. snprintf may itself use significant stack space
     *   3. This string is in .rodata (flash), not on the stack
     *
     * Note: UART might not work if the HAL state is corrupted, but it's
     * worth trying. The LEDs are our reliable fallback.
     */
    const char *err = "\r\n!!! STACK OVERFLOW DETECTED !!!\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)err, 34, 1000);

    /*
     * Infinite loop — system is in a bad state.
     * A watchdog timer (if configured) would eventually reset the MCU.
     * Without a watchdog, the user must manually reset.
     */
    for (;;)
    {
        /* Intentionally empty — just hang here */
    }
}

/**
 * @brief  Idle hook — called whenever the idle task runs.
 *
 * The idle task runs at the lowest priority (priority 0) and executes
 * whenever no other task is ready to run. This is where you'd add:
 *
 *   - __WFI() (Wait For Interrupt) for low-power mode
 *   - Background housekeeping (statistics collection, etc.)
 *   - Runtime checks or assertions
 *
 * IMPORTANT CONSTRAINTS:
 *   - Must NOT block (no vTaskDelay, no semaphore takes)
 *   - Must NOT call any API function that could block
 *   - Should be very fast (it runs in a tight loop)
 *
 * Currently empty — __WFI() is commented out because:
 *   1. It complicates debugging (SWD may disconnect in sleep)
 *   2. Power optimization is not our priority for this demo
 *   3. The DISC1 is USB-powered anyway
 */
void vApplicationIdleHook(void)
{
    /*
     * Uncomment for low-power mode:
     * __WFI();
     *
     * WFI (Wait For Interrupt) puts the CPU into sleep mode until
     * the next interrupt (SysTick, EXTI, DMA, etc.). This can reduce
     * power consumption by 30-50% depending on the MCU and peripherals.
     */
}

/**
 * @brief  Malloc failed hook — called when pvPortMalloc() fails.
 *
 * This indicates FreeRTOS ran out of heap. In our design, all FreeRTOS
 * objects (queues, semaphores, tasks) are created in main() before the
 * scheduler starts. If this hook fires, our heap is too small.
 *
 * Note: This hook is only called if configUSE_MALLOC_FAILED_HOOK = 1.
 * We include it defensively.
 */
void vApplicationMallocFailedHook(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_SET);

    const char *err = "\r\n!!! MALLOC FAILED !!!\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)err, 24, 1000);

    for (;;)
    {
        /* Intentionally empty */
    }
}
