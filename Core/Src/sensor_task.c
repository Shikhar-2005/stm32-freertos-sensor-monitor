/**
  ******************************************************************************
  * @file           : sensor_task.c
  * @brief          : Sensor acquisition task — reads gyroscope + temperature
  *
  * Architecture:
  *   This task is the data PRODUCER. It reads hardware sensors at fixed
  *   intervals and publishes SensorData_t packets to the sensorQueue.
  *   The display task is the CONSUMER.
  *
  *   Using a queue decouples production from consumption:
  *   - If display is slow, sensor data buffers (up to 10 packets)
  *   - If display is fast, it blocks until new data arrives
  *   - No shared memory, no race conditions, no locks needed
  *
  * Timing:
  *   - Gyroscope: every 100ms (10 Hz) — fast enough for motion detection
  *   - Temperature: every 500ms (2 Hz) — temperature changes slowly
  *   - We use a cycle counter (read_count) to interleave these rates
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sensor_task.h"
#include "main.h"
#include "l3gd20.h"
#include <stdio.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/

/** @brief Sensor task loop period in milliseconds */
#define SENSOR_PERIOD_MS        100U

/**
 * @brief How many gyro cycles between temperature readings.
 *
 * 500ms / 100ms = 5 cycles. Temperature is read every 5th iteration.
 * This avoids starting the ADC every cycle — the internal temp sensor
 * has ~1°C accuracy anyway, so high-frequency sampling is pointless.
 */
#define TEMP_READ_INTERVAL      5U

/**
 * @brief ADC to temperature conversion constants.
 *
 * From STM32F429 reference manual (RM0090), section 13.10:
 *   V_SENSE = raw_adc × (V_REF / 4096)
 *   Temperature (°C) = ((V_SENSE - V_25) / Avg_Slope) + 25
 *
 * Where:
 *   V_REF = 3.3V (VDDA on DISC1)
 *   V_25 = 0.76V (typical voltage at 25°C)
 *   Avg_Slope = 2.5 mV/°C = 0.0025 V/°C
 *
 * Combined formula:
 *   temp = ((raw × 3.3 / 4096.0) - 0.76) / 0.0025 + 25.0
 */
#define ADC_VREF                3.3f
#define ADC_RESOLUTION          4096.0f
#define TEMP_V25                0.76f
#define TEMP_AVG_SLOPE          0.0025f
#define TEMP_OFFSET             25.0f

/* Private variables ---------------------------------------------------------*/

/**
 * @brief Tracks whether L3GD20 is responding.
 *
 * If the initial WHO_AM_I check fails or SPI reads start failing,
 * we fall back to dummy data instead of crashing. This lets the rest
 * of the system (display, events) continue working for debugging.
 */
static uint8_t gyro_available = 0;

/** @brief Monotonic counter for generating dummy data when gyro fails */
static uint16_t dummy_counter = 0;

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Read internal temperature sensor via ADC1 Channel 18.
 *
 * This is a blocking read — HAL_ADC_PollForConversion waits until
 * the conversion completes. At 12-bit resolution with default sample
 * time, this takes ~15 ADC cycles, which is negligible at 180 MHz.
 *
 * @retval Temperature in degrees Celsius (float)
 */
static float ReadInternalTemperature(void)
{
    float temperature = 0.0f;
    uint32_t raw_adc = 0;

    /* Start ADC conversion */
    HAL_ADC_Start(&hadc1);

    /* Wait for conversion to complete (timeout: 100ms — extremely generous) */
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    {
        raw_adc = HAL_ADC_GetValue(&hadc1);

        /*
         * Convert raw ADC value to temperature.
         * This formula is from the STM32F4 reference manual.
         * Accuracy is ±1.5°C — fine for monitoring, not for medical use.
         */
        float v_sense = (float)raw_adc * ADC_VREF / ADC_RESOLUTION;
        temperature = ((v_sense - TEMP_V25) / TEMP_AVG_SLOPE) + TEMP_OFFSET;
    }

    HAL_ADC_Stop(&hadc1);

    return temperature;
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Sensor acquisition task main function.
 *
 * Design decisions:
 *   1. vTaskDelayUntil() instead of vTaskDelay(): ensures consistent 100ms
 *      period regardless of how long the SPI/ADC reads take. vTaskDelay()
 *      would drift by the execution time each cycle.
 *
 *   2. Red LED heartbeat: visual confirmation that this task is running.
 *      If the LED stops blinking, the task is stuck or crashed.
 *
 *   3. Dummy data fallback: if the gyroscope fails, we generate incrementing
 *      values instead of crashing. This helps distinguish "SPI broken" from
 *      "task crashed" during debugging.
 *
 * @param  argument  Not used
 */
void SensorTask(void *argument)
{
    (void)argument;  /* Suppress unused parameter warning */

    SensorData_t data;
    TickType_t xLastWakeTime;
    uint32_t read_count = 0;

    /* Print startup banner (helps verify task is actually running) */
    const char *msg = "[SensorTask] Started\r\n";
    if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        xSemaphoreGive(uartMutex);
    }

    /*
     * Check if gyroscope is alive by verifying WHO_AM_I.
     * We do this here (not in main) as a runtime check — the gyro
     * might have been OK at boot but failed later.
     */
    uint8_t id = L3GD20_ReadID();
    if (id == L3GD20_WHO_AM_I_VALUE || id == L3GD20H_WHO_AM_I_VALUE)
    {
        gyro_available = 1;
    }
    else
    {
        gyro_available = 0;
        const char *err = "[SensorTask] WARNING: L3GD20 not detected, using dummy data\r\n";
        if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            HAL_UART_Transmit(&huart1, (uint8_t *)err, strlen(err), HAL_MAX_DELAY);
            xSemaphoreGive(uartMutex);
        }
    }

    /* Initialize temperature to a sane default */
    data.temperature_c = 25.0f;

    /*
     * Initialize xLastWakeTime to current tick count.
     * vTaskDelayUntil() uses this as the reference point for calculating
     * the next wake time. It gets updated automatically each call.
     */
    xLastWakeTime = xTaskGetTickCount();

    /* ---- Main task loop ---- */
    for (;;)
    {
        /* === Read Gyroscope (every cycle = every 100ms) === */
        if (gyro_available)
        {
            HAL_StatusTypeDef status;
            status = L3GD20_ReadXYZ(&data.gyro_x, &data.gyro_y, &data.gyro_z);

            if (status != HAL_OK)
            {
                /*
                 * SPI read failed — fall back to dummy data.
                 * Don't disable gyro_available permanently; it might be
                 * a transient error (e.g., bus contention).
                 */
                data.gyro_x = dummy_counter;
                data.gyro_y = dummy_counter + 100;
                data.gyro_z = dummy_counter + 200;
                dummy_counter++;
            }
        }
        else
        {
            /*
             * Gyro not available — generate dummy incrementing values.
             * Using incrementing values (not zeros) makes it obvious in
             * the UART output that the sensor task IS running, just not
             * reading real hardware.
             */
            data.gyro_x = dummy_counter;
            data.gyro_y = dummy_counter + 100;
            data.gyro_z = dummy_counter + 200;
            dummy_counter++;
        }

        /* === Read Temperature (every 5th cycle = every 500ms) === */
        if ((read_count % TEMP_READ_INTERVAL) == 0)
        {
            data.temperature_c = ReadInternalTemperature();
        }

        /* === Timestamp and send === */
        data.timestamp_ms = xTaskGetTickCount();

        /*
         * Send to queue with zero timeout (non-blocking).
         * If the queue is full, the oldest data is effectively lost.
         * This is acceptable — sensor data is perishable, and we'd
         * rather send fresh data than block the sensor task waiting
         * for the display to catch up.
         */
        xQueueSend(sensorQueue, &data, 0);

        /* === Heartbeat LED === */
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

        /* Increment cycle counter (wraps at UINT32_MAX, which is fine) */
        read_count++;

        /*
         * Wait until next period. This automatically adjusts for the
         * time spent reading sensors, so the period is always exactly
         * 100ms (not 100ms + execution_time like vTaskDelay would be).
         */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}
