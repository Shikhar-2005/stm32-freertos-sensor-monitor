/**
  ******************************************************************************
  * @file           : event_task.h
  * @brief          : Button event handler task header
  *
  * The event task handles button press events. It waits on a binary
  * semaphore (given from the EXTI0 ISR), debounces the button, and
  * forwards validated events to the display task.
  ******************************************************************************
  */

#ifndef __EVENT_TASK_H
#define __EVENT_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  FreeRTOS task function for button event handling.
 *
 * This task runs at osPriorityHigh because button events need fast response.
 * The architecture is: ISR → semaphore → this task → event queue → display.
 * This deferred interrupt pattern keeps the ISR minimal (just a semaphore give)
 * while still achieving low-latency event processing.
 *
 * @param  argument  Not used (NULL). Required by FreeRTOS task signature.
 */
void EventTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __EVENT_TASK_H */
