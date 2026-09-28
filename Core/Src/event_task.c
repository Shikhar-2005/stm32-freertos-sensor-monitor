/**
  ******************************************************************************
  * @file           : event_task.c
  * @brief          : Button event handler task — debounces and processes presses
  *
  * Architecture (Deferred Interrupt Processing):
  *
  *   [Hardware] EXTI0 IRQ fires on PA0 rising edge
  *       ↓
  *   [ISR] xSemaphoreGiveFromISR(buttonSem) — takes ~100ns
  *       ↓
  *   [EventTask] xSemaphoreTake(buttonSem) — wakes up
  *       ↓
  *   [EventTask] Debounce: wait 50ms, re-check pin state
  *       ↓
  *   [EventTask] If valid: toggle LED, send event to display queue
  *
  * Why not handle everything in the ISR?
  *   1. ISRs should be as short as possible (<1µs ideally) to avoid:
  *      - Missing other interrupts
  *      - Starving the FreeRTOS tick interrupt
  *      - Unpredictable timing in the rest of the system
  *
  *   2. You CAN'T call most FreeRTOS API functions from an ISR (only
  *      the "FromISR" variants). No delays, no blocking, no mutex takes.
  *
  *   3. Debouncing requires waiting — which is impossible in an ISR.
  *
  *   4. UART transmission blocks for milliseconds — absolutely forbidden
  *      in an ISR context.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "event_task.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/

/**
 * @brief Debounce delay in milliseconds.
 *
 * Mechanical buttons "bounce" — the contacts make and break rapidly
 * for 5-50ms after a press or release. Without debouncing, a single
 * button press generates multiple interrupts.
 *
 * 50ms is a good balance: long enough to filter all bouncing, short
 * enough that the user perceives instant response.
 */
#define DEBOUNCE_DELAY_MS  50U

/* Priority inversion bug demonstration */
#ifdef BUG_PRIORITY_INVERSION
/**
 * @brief INTENTIONAL BUG: Priority inversion demonstration.
 *
 * Priority inversion occurs when:
 *   1. Low-priority task (L) acquires a resource (semaphore/mutex)
 *   2. High-priority task (H) tries to acquire the same resource → blocks
 *   3. Medium-priority task (M) preempts L (because M > L)
 *   4. H is effectively blocked by M, even though H > M
 *
 * FreeRTOS MUTEX has priority inheritance: when H blocks on a mutex held
 * by L, L's priority is temporarily raised to H's level. This prevents
 * M from preempting L, so L finishes quickly and releases the mutex.
 *
 * Binary SEMAPHORE does NOT have priority inheritance. Using a binary
 * semaphore as a mutex causes real priority inversion.
 *
 * HOW TO TRIGGER:
 *   1. Define BUG_PRIORITY_INVERSION in build settings
 *   2. Create 3 tasks at Low, Med, High priority sharing a binary semaphore
 *   3. Low task takes semaphore, does long computation
 *   4. High task blocks on semaphore
 *   5. Med task runs instead of High → that's the inversion
 *
 * THE FIX: Use xSemaphoreCreateMutex() instead of xSemaphoreCreateBinary()
 * for mutual exclusion. Mutexes have priority inheritance built in.
 */
#warning "BUG_PRIORITY_INVERSION is enabled — demonstrating priority inversion"

/*
 * Shared semaphore without priority inheritance.
 * In production, this should be a mutex (xSemaphoreCreateMutex).
 */
static SemaphoreHandle_t shared_resource_sem;

static void PriorityInversionDemo(void)
{
    /* Simulate: High-priority task tries to take a resource held by low-priority task */
    /* The binary semaphore has no priority inheritance, so medium-priority tasks */
    /* can preempt the low-priority task and delay high-priority task indefinitely */
    shared_resource_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(shared_resource_sem);
    /* Low priority task would take this, do long work, while high priority waits */
}
#endif

/* Missing volatile bug demonstration */
#ifdef BUG_NO_VOLATILE
/**
 * @brief INTENTIONAL BUG: Missing volatile demonstration.
 *
 * When a variable is shared between an ISR and a task (or between tasks
 * on different cores), it MUST be declared volatile. Without volatile,
 * the compiler may:
 *   - Cache the variable in a CPU register (task never sees ISR's updates)
 *   - Reorder reads/writes (breaking expected sequence)
 *   - Optimize away "redundant" reads (if the task reads it in a loop)
 *
 * HOW TO TRIGGER:
 *   1. Define BUG_NO_VOLATILE in build settings
 *   2. Compile with -O2 optimization
 *   3. The ISR increments shared_counter, but the task loop never sees
 *      the update because the compiler cached it in a register
 *
 * THE FIX: Add the volatile qualifier:
 *   volatile uint32_t shared_counter = 0;
 */
#warning "BUG_NO_VOLATILE is enabled — shared_counter may not update visibly"

/* BUG: This should be volatile! */
uint32_t shared_counter = 0;  /* ISR increments, task reads — compiler may cache */
#endif

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Event task main function.
 *
 * Design decisions:
 *   1. Binary semaphore (not counting): We only care about "at least one
 *      press happened," not "exactly N presses happened." If the user
 *      presses rapidly during debounce, we process one event and ignore
 *      the rest. This is the expected UX behavior.
 *
 *   2. Priority osPriorityHigh: Button response should feel instant.
 *      This task preempts the sensor and display tasks to process
 *      the button event as quickly as possible.
 *
 *   3. Re-check pin state after debounce: This filters out electrical
 *      noise that might trigger EXTI without a real button press.
 *      If PA0 is no longer high after 50ms, it was noise.
 *
 * @param  argument  Not used
 */
void EventTask(void *argument)
{
    (void)argument;

    EventData_t event;

    /* Print startup banner */
    const char *msg = "[EventTask] Started\r\n";
    if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        xSemaphoreGive(uartMutex);
    }

    /* ---- Main task loop ---- */
    for (;;)
    {
        /*
         * Block until the button ISR gives the semaphore.
         * portMAX_DELAY = wait forever. This task uses zero CPU while waiting.
         *
         * A binary semaphore starts "empty" (taken). The ISR "gives" it
         * (sets it to "full"), which unblocks this xSemaphoreTake.
         * Then it's "empty" again, ready for the next ISR event.
         */
        if (xSemaphoreTake(buttonSem, portMAX_DELAY) == pdTRUE)
        {
            /*
             * Debounce: wait 50ms for mechanical bouncing to settle.
             * Using vTaskDelay (not vTaskDelayUntil) because we don't need
             * precise periodic timing here — we just need a delay.
             */
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));

            /*
             * Re-check button state: PA0 should still be HIGH if this
             * was a real press (user is still holding the button, or it
             * settled in the pressed state).
             *
             * Note: The STM32F429I-DISC1 user button is active HIGH
             * (pressed = VDD, released = GND), no external pull-down needed.
             */
            if (HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_SET)
            {
                /* === Valid button press confirmed === */

                /* Toggle green LED as user feedback */
                HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

                /* Package event data */
                event.event_type = 0;  /* 0 = button press */
                event.timestamp_ms = xTaskGetTickCount();

                /*
                 * Send event to display task's event queue.
                 * Non-blocking (timeout = 0): if the queue is full,
                 * drop the event rather than blocking this high-priority task.
                 */
                xQueueSend(eventQueue, &event, 0);

                /* Print immediate confirmation via UART */
                const char *evt_msg = "[EVENT] Button ISR -> Semaphore -> Task\r\n";
                if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    HAL_UART_Transmit(&huart1, (uint8_t *)evt_msg, strlen(evt_msg), HAL_MAX_DELAY);
                    xSemaphoreGive(uartMutex);
                }
            }
            /* else: noise or bounce — ignore silently */
        }
    }
}
