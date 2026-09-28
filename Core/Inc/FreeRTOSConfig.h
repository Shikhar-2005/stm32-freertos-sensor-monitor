/**
  ******************************************************************************
  * @file           : FreeRTOSConfig.h
  * @brief          : FreeRTOS kernel configuration for STM32F429
  *
  * This file configures FreeRTOS for the STM32F429ZIT6 running at 180 MHz.
  * In a CubeMX-generated project, these settings come from the CubeMX UI.
  * Here they are set manually for clarity and documentation.
  *
  * Key design decisions:
  *   - Preemptive scheduler (tasks can be interrupted by higher priority)
  *   - 1ms tick rate (standard, gives 1ms timing resolution)
  *   - Stack overflow checking enabled (method 2 — pattern checking)
  *   - Priority inheritance mutexes enabled
  *   - Static allocation disabled (using heap_4 dynamic allocation)
  ******************************************************************************
  */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *----------------------------------------------------------*/

/* Cortex-M4 specific: use hardware features for better performance */
#define configUSE_PREEMPTION                    1

/**
 * CPU clock frequency.
 * Must match SystemClock_Config() output.
 * FreeRTOS uses this to calculate SysTick reload value.
 */
#define configCPU_CLOCK_HZ                      ( 180000000UL )

/**
 * Tick rate = 1000 Hz = 1ms per tick.
 * This determines the resolution of vTaskDelay and timing.
 * Higher values give finer resolution but more interrupt overhead.
 * 1000 Hz is the standard for most embedded applications.
 */
#define configTICK_RATE_HZ                      ( (TickType_t)1000 )

/**
 * Maximum number of priority levels.
 * We use up to osPriorityHigh (40+), so 56 covers all CMSIS-RTOS2 mappings.
 * RAM cost: ~56 bytes for the ready list array (trivial).
 */
#define configMAX_PRIORITIES                    ( 56 )

/**
 * Minimum stack size for any task, in WORDS (not bytes!).
 * 128 words = 512 bytes. Used for the idle task.
 * Our custom tasks specify their own (larger) stack sizes.
 */
#define configMINIMAL_STACK_SIZE                ( (uint16_t)128 )

/**
 * Total FreeRTOS heap size in bytes.
 * Must be large enough for all tasks, queues, semaphores, mutexes.
 *
 * Rough calculation:
 *   Task stacks:  512 + 512 + 256 = 1280 words = 5120 bytes
 *   Task TCBs:    3 × ~100 bytes = 300 bytes
 *   Queues:       10 × sizeof(SensorData_t) + 5 × sizeof(EventData_t) ≈ 200 bytes
 *   Semaphores:   2 × ~80 bytes = 160 bytes
 *   Idle task:    128 words = 512 bytes + TCB
 *   Timer task:   configTIMER_TASK_STACK_DEPTH × 4 + TCB
 *   Overhead:     ~500 bytes
 *   Total:        ~8000 bytes
 *
 * 15360 bytes gives comfortable headroom.
 */
#define configTOTAL_HEAP_SIZE                   ( (size_t)15360 )

/** Maximum length of task name strings (including null terminator) */
#define configMAX_TASK_NAME_LEN                 ( 16 )

/** Use 32-bit tick counter (wraps after ~49 days at 1ms ticks) */
#define configUSE_16_BIT_TICKS                  0

/**
 * Allow idle task to yield to other priority-0 tasks.
 * Not critical since we don't have other priority-0 tasks,
 * but it's good practice to leave this enabled.
 */
#define configIDLE_SHOULD_YIELD                 1

/* ---- Feature enables ---- */

/** Enable mutex support (required for uartMutex with priority inheritance) */
#define configUSE_MUTEXES                       1

/** Enable recursive mutexes (not used, but doesn't hurt) */
#define configUSE_RECURSIVE_MUTEXES             1

/** Enable counting semaphores (not used, but available) */
#define configUSE_COUNTING_SEMAPHORES           1

/** Enable queue sets (not used, but available) */
#define configQUEUE_REGISTRY_SIZE               8

/* ---- Hook function configuration ---- */

/**
 * Enable idle hook (vApplicationIdleHook).
 * Called whenever the idle task runs. We use it for optional __WFI().
 */
#define configUSE_IDLE_HOOK                     1

/**
 * Enable tick hook (vApplicationTickHook).
 * Called every tick. We don't need it, so disabled.
 */
#define configUSE_TICK_HOOK                     0

/**
 * Enable malloc failure hook (vApplicationMallocFailedHook).
 * Called when pvPortMalloc fails. Critical for debugging heap issues.
 */
#define configUSE_MALLOC_FAILED_HOOK            1

/**
 * Enable stack overflow detection.
 * Method 2 (= 2): fills stack with pattern 0xA5 at creation, checks
 * last 20 bytes on every context switch. Catches most overflows.
 * Method 1 (= 1): only checks if SP went below stack start. Cheaper but
 * misses overflows that don't corrupt the very end of the stack.
 */
#define configCHECK_FOR_STACK_OVERFLOW          2

/* ---- Runtime stats (disabled for now) ---- */
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* ---- Co-routine configuration (not used) ---- */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )

/* ---- Software timer configuration ---- */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( 2 )
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             ( configMINIMAL_STACK_SIZE * 2 )

/* ---- Interrupt nesting configuration ---- */

/**
 * Cortex-M interrupt priority configuration.
 *
 * STM32F4 uses 4 bits of interrupt priority (0-15, where 0 is highest).
 * FreeRTOS needs to know which priority levels are safe for its API calls.
 *
 * configLIBRARY_LOWEST_INTERRUPT_PRIORITY:
 *   Lowest priority level (15). Used for PendSV and SysTick.
 *
 * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY:
 *   Highest priority from which FreeRTOS API calls are safe (5).
 *   ISRs with priority 0-4 CANNOT call any FreeRTOS functions.
 *   ISRs with priority 5-15 CAN call "FromISR" functions.
 *
 * Our EXTI0 ISR is at priority 6, which is in the safe zone.
 */
#ifdef __NVIC_PRIO_BITS
  #define configPRIO_BITS   __NVIC_PRIO_BITS
#else
  #define configPRIO_BITS   4  /* STM32F4xx uses 4 priority bits */
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY     5

/* Internal FreeRTOS representation (shifted to MSB position) */
#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* ---- FreeRTOS API function includes ---- */

/**
 * Set to 1 to include the API function, 0 to exclude.
 * Excluding unused functions saves a small amount of code space.
 */
#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_xTaskGetSchedulerState      1
#define INCLUDE_uxTaskGetStackHighWaterMark  1
#define INCLUDE_xTimerPendFunctionCall       1
#define INCLUDE_eTaskGetState               1

/* ---- Cortex-M4 specific handler names ---- */

/**
 * Map FreeRTOS handler names to CMSIS names.
 * FreeRTOS port implements these handlers:
 *   - xPortPendSVHandler: context switching (PendSV)
 *   - xPortSysTickHandler: tick increment (SysTick)
 *   - vPortSVCHandler: start first task (SVC)
 *
 * These #defines ensure the vector table uses the FreeRTOS implementations
 * instead of the default weak stubs.
 */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/* ---- CMSIS-RTOS2 compatibility ---- */

/**
 * CMSIS-RTOS2 priority mapping.
 *
 * CubeMX generates code using osPriorityXxx constants.
 * These map to FreeRTOS numeric priorities.
 *
 * Note: FreeRTOS priorities are 0 (lowest) to configMAX_PRIORITIES-1 (highest).
 * CMSIS-RTOS2 priorities are centered around osPriorityNormal = 24.
 */
#ifndef osPriorityNormal
#define osPriorityNormal        24
#endif
#ifndef osPriorityAboveNormal
#define osPriorityAboveNormal   32
#endif
#ifndef osPriorityHigh
#define osPriorityHigh          40
#endif

#endif /* FREERTOS_CONFIG_H */
