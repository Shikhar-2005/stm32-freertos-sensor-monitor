/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  *
  * Project: FreeRTOS Multi-Sensor Monitor
  * Target:  STM32F429ZIT6 (STM32F429I-DISC1)
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Sensor data packet sent from SensorTask → DisplayTask via queue.
 *
 * Design decision: Using a struct keeps all sensor readings atomic —
 * the display task always gets a consistent snapshot of all sensors
 * at the same timestamp, rather than potentially mixing old/new readings.
 */
typedef struct {
    int16_t  gyro_x;        /**< Raw gyroscope X-axis (L3GD20) */
    int16_t  gyro_y;        /**< Raw gyroscope Y-axis (L3GD20) */
    int16_t  gyro_z;        /**< Raw gyroscope Z-axis (L3GD20) */
    float    temperature_c; /**< Internal temperature sensor (°C) */
    uint32_t timestamp_ms;  /**< Tick count at time of reading */
} SensorData_t;

/**
 * @brief Event data packet sent from EventTask → DisplayTask via queue.
 *
 * Design decision: Using a separate event queue (not the sensor queue)
 * avoids priority inversion and keeps event latency low — events don't
 * have to wait behind buffered sensor data.
 */
typedef struct {
    uint8_t  event_type;    /**< Event type: 0 = button press */
    uint32_t timestamp_ms;  /**< Tick count when event was processed */
} EventData_t;

/* Exported constants --------------------------------------------------------*/

/* LED Pin Definitions — STM32F429I-DISC1 onboard LEDs on GPIOG */
#define LED_GREEN_Pin        GPIO_PIN_13
#define LED_GREEN_GPIO_Port  GPIOG
#define LED_RED_Pin          GPIO_PIN_14
#define LED_RED_GPIO_Port    GPIOG

/* User Button — PA0, active high with EXTI0 */
#define USER_BTN_Pin         GPIO_PIN_0
#define USER_BTN_GPIO_Port   GPIOA

/* L3GD20 SPI Chip Select — PC1, active low */
#define GYRO_CS_Pin          GPIO_PIN_1
#define GYRO_CS_GPIO_Port    GPIOC

/* Exported variables --------------------------------------------------------*/

/*
 * FreeRTOS synchronization objects — declared extern here so all tasks
 * and ISRs can access them. Defined in main.c.
 *
 * Why globals? FreeRTOS handles are inherently shared resources.
 * Making them global with extern is the standard embedded pattern.
 */
extern QueueHandle_t     sensorQueue;   /**< SensorTask → DisplayTask */
extern QueueHandle_t     eventQueue;    /**< EventTask  → DisplayTask */
extern SemaphoreHandle_t buttonSem;     /**< ISR → EventTask (binary) */
extern SemaphoreHandle_t uartMutex;     /**< Mutex for UART access    */

extern SPI_HandleTypeDef  hspi5;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef  hadc1;

/* Exported functions prototypes ---------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_SPI5_Init(void);
void MX_ADC1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
