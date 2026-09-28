/**
  ******************************************************************************
  * @file           : display_task.c
  * @brief          : UART display output task — formats and prints sensor data
  *
  * Architecture:
  *   This task is the data CONSUMER. It blocks on the sensor queue,
  *   formats received data into human-readable strings, and transmits
  *   via UART. It also polls the event queue for button press events.
  *
  * UART Mutex:
  *   Multiple tasks may want to print (event task, sensor task for errors).
  *   The uartMutex prevents interleaved output. FreeRTOS mutexes have
  *   built-in priority inheritance, so holding this mutex won't cause
  *   priority inversion.
  *
  * Buffer Sizing:
  *   We use snprintf (not sprintf) with a fixed 200-byte buffer.
  *   The longest possible output line is ~80 chars, so 200 bytes gives
  *   comfortable margin without risking stack overflow.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "display_task.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/

/**
 * @brief Output buffer size in bytes.
 *
 * Sized to hold the longest possible formatted string with margin.
 * Max line: "\r\n[99999 ms] Gyro: X=-32768 Y=-32768 Z=-32768 | Temp: -99.9 C\r\n"
 * That's about 70 chars. 200 bytes is very safe.
 */
#define DISPLAY_BUF_SIZE  200U

/* Stack overflow bug demonstration */
#ifdef BUG_STACK_OVERFLOW
/**
 * @brief INTENTIONAL BUG: Stack overflow demonstration.
 *
 * With a 512-word (2048-byte) task stack, allocating a 1024-byte buffer
 * on the stack (plus local variables, saved registers, etc.) will exceed
 * the available stack space and trigger vApplicationStackOverflowHook().
 *
 * HOW TO TRIGGER:
 *   1. Define BUG_STACK_OVERFLOW in your build settings
 *   2. Flash and run
 *   3. Both LEDs will turn on solid (error indicator from the hook)
 *
 * WHY THIS HAPPENS:
 *   FreeRTOS allocates a fixed-size stack per task. Unlike desktop apps,
 *   there's no virtual memory or stack growth. If you exceed the stack,
 *   you corrupt the TCB (Task Control Block) or adjacent memory.
 *
 * THE FIX:
 *   - Use snprintf with a reasonably-sized buffer (200 bytes)
 *   - Or increase the task's stack size
 *   - configCHECK_FOR_STACK_OVERFLOW = 2 catches this (but only if enabled!)
 */
#warning "BUG_STACK_OVERFLOW is enabled — DisplayTask WILL crash!"
#endif

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Display task main function.
 *
 * Design decisions:
 *   1. Blocking receive on sensorQueue: this task naturally wakes up
 *      whenever new sensor data arrives (every 100ms). No need for a
 *      separate timer or polling loop.
 *
 *   2. Non-blocking check on eventQueue: events are rare (button presses).
 *      We don't want to block on the event queue because that would delay
 *      sensor display. Instead, we check it opportunistically each time
 *      we wake up for sensor data.
 *
 *   3. snprintf for safety: even though we calculated the buffer is large
 *      enough, using snprintf is defense-in-depth. A future code change
 *      might add longer format strings, and snprintf will truncate instead
 *      of causing a buffer overflow.
 *
 * @param  argument  Not used
 */
void DisplayTask(void *argument)
{
    (void)argument;

    SensorData_t sensor_data;
    EventData_t  event_data;
    char buf[DISPLAY_BUF_SIZE];
    int len;

    /* Print startup banner */
    const char *msg = "[DisplayTask] Started\r\n";
    if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        xSemaphoreGive(uartMutex);
    }

    /* ---- Main task loop ---- */
    for (;;)
    {
#ifdef BUG_STACK_OVERFLOW
        /*
         * INTENTIONAL BUG: 1024-byte stack allocation + sprintf (unbounded).
         * This WILL overflow the 2048-byte task stack.
         */
        char huge_buf[1024];
        sprintf(huge_buf, "This buffer is way too large for our stack! "
                "The task stack is only 2048 bytes, and this buffer alone "
                "is 1024 bytes. Add in saved registers, return addresses, "
                "and local variables, and we're way over the limit.");
        /* We'll never get here — stack overflow hook fires first */
        HAL_UART_Transmit(&huart1, (uint8_t *)huge_buf, strlen(huge_buf), HAL_MAX_DELAY);
#endif

        /*
         * Block until sensor data arrives. portMAX_DELAY means wait forever.
         * This is efficient — the task consumes zero CPU while waiting.
         * The scheduler will run lower-priority tasks or the idle task.
         */
        if (xQueueReceive(sensorQueue, &sensor_data, portMAX_DELAY) == pdTRUE)
        {
            /*
             * Format sensor data into human-readable string.
             *
             * Format breakdown:
             *   %5lu  — timestamp, right-aligned, 5 digits (wraps at ~49 days)
             *   %6d   — gyro values, right-aligned, 6 chars (range: -32768 to 32767)
             *   %.1f  — temperature with 1 decimal place
             *
             * \r\n at start: creates blank line between readings for readability
             * \r\n at end: ensures line is flushed on most terminals
             */
            len = snprintf(buf, sizeof(buf),
                           "\r\n[%5lu ms] Gyro: X=%6d Y=%6d Z=%6d | Temp: %.1f C\r\n",
                           (unsigned long)sensor_data.timestamp_ms,
                           sensor_data.gyro_x,
                           sensor_data.gyro_y,
                           sensor_data.gyro_z,
                           sensor_data.temperature_c);

            /* Acquire UART mutex before transmitting */
            if (xSemaphoreTake(uartMutex, portMAX_DELAY) == pdTRUE)
            {
                /*
                 * Clamp len to buffer size to be safe.
                 * snprintf returns what it WOULD have written, which could
                 * exceed the buffer size if the format string was too long.
                 */
                if (len > 0 && len < (int)sizeof(buf))
                {
                    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
                }

                xSemaphoreGive(uartMutex);
            }
        }

        /*
         * Check for button events (non-blocking).
         * Timeout = 0 means: return immediately if queue is empty.
         * This way we don't delay sensor data display waiting for
         * a button press that may never come.
         */
        if (xQueueReceive(eventQueue, &event_data, 0) == pdTRUE)
        {
            len = snprintf(buf, sizeof(buf),
                           "[EVENT] Button pressed at %lu ms\r\n",
                           (unsigned long)event_data.timestamp_ms);

            if (xSemaphoreTake(uartMutex, portMAX_DELAY) == pdTRUE)
            {
                if (len > 0 && len < (int)sizeof(buf))
                {
                    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
                }
                xSemaphoreGive(uartMutex);
            }
        }
    }
}
