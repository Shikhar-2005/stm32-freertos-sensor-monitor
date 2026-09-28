/**
  ******************************************************************************
  * @file           : display_task.h
  * @brief          : Display/UART output task header
  *
  * The display task receives sensor data and events from queues,
  * formats them into human-readable strings, and outputs via UART.
  ******************************************************************************
  */

#ifndef __DISPLAY_TASK_H
#define __DISPLAY_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  FreeRTOS task function for UART display output.
 *
 * This task runs at osPriorityNormal (lower than sensor and event tasks)
 * because display is the least time-critical operation. It blocks on the
 * sensor queue and also polls the event queue non-blocking each iteration.
 *
 * @param  argument  Not used (NULL). Required by FreeRTOS task signature.
 */
void DisplayTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_TASK_H */
