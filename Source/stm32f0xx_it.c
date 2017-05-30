/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx_it.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    uint8_t i = 0;
    while(i < N_TIMERS) {
        if (TimersStr.flagCountEnable[i]) {
            TimersStr.counter[i]++;
        }
        i++;
    }
    //uint8_t data[2];
    //I2C_Read_Transaction (0xA0, 0x00, &data[0], 2);
    //I2C_Read_Transaction (0xAA, 0x00, &data[0], 2);
    //I2C_Write_Transaction (0x55, 0x55, &data[0], 2);
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

//***************************************************************************
//Обработчик прерывания для измерения RPM компрессора
//***************************************************************************
void EXTI4_15_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line12) != RESET) {
    TIM15->CR1 = TIM_CR1_CEN;      //запустить таймер
    CompresStr.countTacho++;
    if((CompresStr.countTacho > 6) && (TIM15->CR1 & TIM_CR1_CEN)) {
        CompresStr.timePeriodTurn = TIM15->CNT; // запоминаем время
        TIM15->CR1 &= ~TIM_CR1_CEN;             // останавливаем таймер
        TIM15->CNT = 0;                         // сбрасываем время таймера
        CompresStr.flagCountTimDone = 1;
        CompresStr.countTacho = 0;
//        NVIC_DisableIRQ(EXTI4_15_IRQn);         // запрещаем прерывания
    }
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
}


//***************************************************************************
//Обработчик прерывания DMA для TACHO1 
//***************************************************************************
//void DMA1_Channel4_5_IRQHandler (void)
//{
//    if (DMA1->ISR & DMA1_FLAG_TC4){         // прерывание по окончанию трансфера
//        DMA1_Channel4->CCR &= ~DMA_CCR_EN;  // выключаем DMA
//        DMA1_Channel4->CNDTR = 2;           // обновляем количество пересылаемых значений
//        //TIM16->CR1 &= ~TIM_CR1_CEN;         // выключаем таймер
//        TIM16->EGR |= TIM_EGR_UG;           // сбрасываем счётчики
//        DMA1->IFCR |= DMA_IFCR_CGIF4;           // сбрасываем все флаги прерывания DMA
//    }   
//}


//***************************************************************************
//Обработчик прерывания DMA для TACHO1(вент. конденс.) и TACHO2(вент. испарит.) 
//***************************************************************************
//void DMA1_Channel2_3_IRQHandler (void)
//{
//    if (DMA1->ISR & DMA1_FLAG_GL3){         // прерывание по окончанию трансфера 
//        DMA1_Channel3->CCR &= ~DMA_CCR_EN;  // выключаем DMA
//        DMA1_Channel3->CNDTR = 2;           // обновляем количество пересылаемых значений
//        TIM16->CR1 &= ~TIM_CR1_CEN;         // выключаем таймер
//        TIM16->EGR |= TIM_EGR_UG;           // сбрасываем счётчики
//        DMA1->IFCR |= DMA_IFCR_CGIF3;       // сбрасываем все флаги прерывания DMA
//    } 
//    if (DMA1->ISR & DMA1_FLAG_GL2){         // прерывание по окончанию трансфера
//        DMA1_Channel2->CCR &= ~DMA_CCR_EN;  // выключаем DMA
//        DMA1_Channel2->CNDTR = 2;           // обновляем количество пересылаемых значений
//        TIM17->CR1 &= ~TIM_CR1_CEN;         // выключаем таймер
//        TIM17->EGR |= TIM_EGR_UG;           // сбрасываем счётчики
//        DMA1->IFCR |= DMA_IFCR_CGIF2;       // сбрасываем все флаги прерывания DMA
//    }
//}



//***************************************************************************
//Обработчик прерывания TIM15
//***************************************************************************
//void TIM15_IRQHandler(void)
//{ 
//    TIM15->SR &= ~TIM_SR_UIF;               //Сбрасываем флаг UIF
//}



//***************************************************************************
//Обработчик прерывания USART1 
//ВНУТРЕННИЙ ModBus
//***************************************************************************
void USART1_IRQHandler(void)
{
    //Receive Data register not empty interrupt
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        modBusInt.rxtimer=0;
        if(modBusInt.rxcnt>(BUF_SZ-2)) {
            modBusInt.rxcnt=0;
        }
        modBusInt.buffer[modBusInt.rxcnt++] = USART_ReceiveData (USART1);
    }
    //Transmission complete interrupt
    if(USART_GetITStatus(USART1, USART_IT_TC) != RESET) {
        USART_ClearITPendingBit(USART1, USART_IT_TC);
        USART_ClearITPendingBit(USART1, USART_IT_FE);
        if(modBusInt.txcnt<modBusInt.txlen) {
            USART_SendData(USART1, modBusInt.buffer[modBusInt.txcnt++]);
//            countErrUSART1 = 0; //сброс счётчика ошибок
        } else {
            USART1->CR1 &= (uint32_t)~((uint32_t)USART_CR1_UE);
            USART1->CR1 |= USART_CR1_UE;
            modBusInt.txlen=0;
            GPIOB->BRR = GPIO_Pin_3;        //RS485 DE disable
            USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
            USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        }
    }
    //Error interrupt.
    if((USART1->ISR & USART_ISR_ORE) || (USART1->ISR & USART_ISR_FE) || \
       (USART1->ISR & USART_ISR_PE)  || (USART2->ISR & USART_ISR_CMF)) {
        volatile uint16_t temp;
        temp = (uint16_t)(USART2->RDR & (uint16_t)0x01FF);
        USART1->ICR |= USART_ICR_ORECF;
        USART1->ICR |= USART_ICR_FECF;
        USART1->ICR |= USART_ICR_PECF;
        USART1->ICR |= USART_ICR_CMCF;
    }
}



//***************************************************************************
//Обработчик прерывания USART2
//ВНЕШНИЙ ModBus
//***************************************************************************
void USART2_IRQHandler(void)
{
    //Receive Data register not empty interrupt
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        modBusExt.rxtimer=0;
        if(modBusExt.rxcnt>(BUF_SZ-2)) {
            modBusExt.rxcnt=0;
        }
        modBusExt.buffer[modBusExt.rxcnt++] = USART_ReceiveData(USART2);
    }
    //Transmission complete interrupt
    if(USART_GetITStatus(USART2, USART_IT_TC) != RESET) {
        USART_ClearITPendingBit(USART2, USART_IT_TC);
        if(modBusExt.txcnt<modBusExt.txlen) {
            USART_SendData(USART2, modBusExt.buffer[modBusExt.txcnt++]);
            modBusExt.countReqTest = 0;         //сброс счётчика запросов
        } else {
            USART2->CR1 &= (uint32_t)~((uint32_t)USART_CR1_UE);
            USART2->CR1 |= USART_CR1_UE;
            modBusExt.txlen=0;
            GPIOA->BRR = GPIO_Pin_1;    //RS485 DE disable

            USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
            USART_ITConfig(USART2, USART_IT_TC, DISABLE);
        }
    }
    //Error interrupt.
    if((USART2->ISR & USART_ISR_ORE) || (USART2->ISR & USART_ISR_FE) || \
        (USART2->ISR & USART_ISR_PE)  || (USART2->ISR & USART_ISR_CMF)){
        volatile uint16_t data_t;
        data_t = (uint16_t)(USART2->RDR & (uint16_t)0x01FF);
        USART2->ICR |= USART_ICR_ORECF;
        USART2->ICR |= USART_ICR_FECF;
        USART2->ICR |= USART_ICR_PECF;
        USART2->ICR |= USART_ICR_CMCF;
    }
}


//***************************************************************************
//Обработчик прерывания TIM14 для определения окончания приёма данных по ModBus
//***************************************************************************
void TIM14_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
	//проверяем окончание приёма uart1
	if((modBusInt.rxtimer++>modBusInt.delay)&(modBusInt.rxcnt>1)){
		//Подготовка данных на отправку
		modbus_slave1(&modBusInt);							
		//Старт отправки данных (НА ВНУТРЕННИЙ) по UART1 если данные готовы
		if((modBusInt.txlen>0)&(modBusInt.txcnt==0)){
			USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
			USART_ITConfig(USART1, USART_IT_TC, ENABLE);
			//rs485 DE enable
			GPIOB->BSRR = GPIO_Pin_3;
			USART_SendData(USART1, modBusInt.buffer[modBusInt.txcnt++]);
		}
	}
	//проверяем окончание приёма uart2
	if((modBusExt.rxtimer++>modBusExt.delay)&(modBusExt.rxcnt>1)){
		//Подготовка данных на отправку
		modbus_slave2(&modBusExt);
		//Старт отправки данных (НА ВНЕШНИЙ) по UART2 если данные готовы
		if((modBusExt.txlen>0)&(modBusExt.txcnt==0)){
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
			USART_ITConfig(USART2, USART_IT_TC, ENABLE);
			//rs485 DE enable
			GPIOA->BSRR = GPIO_Pin_1;
			USART_SendData(USART2, modBusExt.buffer[modBusExt.txcnt++]);
		}
	}
}
