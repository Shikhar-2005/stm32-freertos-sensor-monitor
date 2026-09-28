/**
  ******************************************************************************
  * @file           : stm32f4xx_it.h
  * @brief          : Interrupt handler declarations
  ******************************************************************************
  */

#ifndef __STM32F4XX_IT_H
#define __STM32F4XX_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);

/* Note: SVC_Handler, PendSV_Handler, SysTick_Handler are provided by FreeRTOS */

void EXTI0_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4XX_IT_H */
