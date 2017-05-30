/* Includes ------------------------------------------------------------------*/
#include "f_delays.h"




//Функция задержки в милисекундах
void delay_ms(uint32_t ms)
{
    volatile uint32_t nCount;
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq (&RCC_Clocks);

    nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
    for (; nCount!=0; nCount--);
}
//*****************************************************************************


//Функция задержки в микросекундах
void delay_mks(uint32_t mks)
{
    if (mks==0) mks=1;
    TIM6->ARR = mks+1;                  	        //загрузить значение задержки
    TIM6->CNT = 0;
    TIM6->CR1 = TIM_CR1_CEN | TIM_CR1_OPM;      //запустить таймер в режиме одного импульса
    while((TIM6->SR & TIM_SR_UIF)==0){} 	    //дождаться конца задержки
    TIM6->SR &= ~TIM_SR_UIF;	      			//сбросить флаг
}
//*******************************************************************************
