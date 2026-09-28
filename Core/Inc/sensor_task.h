/**
  ******************************************************************************
  * @file           : sensor_task.h
  * @brief          : Sensor acquisition task header
  *
  * The sensor task is responsible for periodically reading the L3GD20
  * gyroscope (every 100ms) and the internal temperature sensor (every 500ms),
  * packaging the data, and sending it to the display task via a queue.
  ******************************************************************************
  */

#ifndef __SENSOR_TASK_H
#define __SENSOR_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  FreeRTOS task function for sensor acquisition.
 *
 * This task runs at osPriorityAboveNormal because sensor data acquisition
 * has real-time requirements — we need consistent 100ms sampling intervals.
 * Using vTaskDelayUntil() ensures timing isn't affected by task execution time.
 *
 * @param  argument  Not used (NULL). Required by FreeRTOS task signature.
 */
void SensorTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_TASK_H */
