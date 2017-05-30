/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F0XX_IT_H
#define __STM32F0XX_IT_H


/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx.h"
#include "main_ini.h"
#include "td_data.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/*void PPP_IRQHandler(void);*/
void EXTI4_15_IRQHandler(void);
void DMA1_Channel4_5_IRQHandler(void);
void DMA1_Channel2_3_IRQHandler (void);
void TIM15_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void TIM14_IRQHandler(void);

#endif /* __STM32F0XX_IT_H */
