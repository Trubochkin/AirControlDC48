#include "main_ini.h"


//*****************************************************************************
//Функции для инициализации портов вводв/вывода
//*****************************************************************************
//Функция переключает пин в IN
void pin_IN(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOx, &GPIO_InitStruct);
}


//Функция переключает пин в OUT_OD
void pin_OUT_OD(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOx, &GPIO_InitStruct);
}

//Функция переключает пин в OUT_PP
void pin_OUT_PP(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOx, &GPIO_InitStruct);
}


//*****************************************************************************
//Таймер для отсчёта задержек (delay_mks)
//*****************************************************************************
void initializeTimerDelay(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;     //подать тактирование на TIM6
    TIM6->PSC     = 48 - 1;                 //настроить делитель для формирования микросекунд
}


//*****************************************************************************
//Инициализация пинов для компрессора
//*****************************************************************************
void initializeCompress(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM_TimeBaseInitTypeDef timerInitStructure;
  TIM_OCInitTypeDef PWMChannelInit;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2ENR_TIM1EN, ENABLE);
  EXTI_InitTypeDef EXTI_InitStruct;

  // Инициализация пина для RPM компрессора
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;	
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_2);

  RCC_APB2PeriphClockCmd(RCC_APB2ENR_SYSCFGEN, ENABLE);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource12);
  EXTI_InitStruct.EXTI_Line = EXTI_Line12;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStruct);

  NVIC_SetPriority(EXTI4_15_IRQn, 0);                    //задаем приоритет прерывания
  NVIC_EnableIRQ(EXTI4_15_IRQn);                         //Разрешение EXTI4_15_IRQn прерывания 

// Инициализация пина для ШИМ компрессора
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;	
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);

  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  // Запускаем таймер на тактовой частоте в 8 mHz
  timerInitStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 8000000) - 1;
  // Период => TIM_Clock/TIM_Period = frequency PWM (Hz)  (duty 8 kHz)
  timerInitStructure.TIM_Period = 1000-1;
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  TIM_TimeBaseInit(TIM1, &timerInitStructure);
  TIM_Cmd(TIM1, ENABLE);
    
  PWMChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
  PWMChannelInit.TIM_Pulse = 1000;   //duty default (reverse control)
  PWMChannelInit.TIM_OutputState = TIM_OutputState_Enable;
  PWMChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC4Init(TIM1, &PWMChannelInit);
  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
}

//*****************************************************************************
//Таймер для измерения одного периода оборота компрессора
//*****************************************************************************
void initializeTimerTachoCompres(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;        //подать тактирование на TIM6
    TIM15->CR1 = TIM_CR1_OPM;                   //счёт до первого переполнения
    TIM15->PSC     = 480 - 1;                   //инкремент счётчика каждые 10 микросекунд
}


//*****************************************************************************
//Инициализация АЦП
//*****************************************************************************
void initializeADC(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    /* Configure ADC Channel0 and Channel4 as analog input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
    /* DMA1 Channel1 Config */
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCStr.buffData[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 41;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1,&DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1,ENABLE);
    
    /* ADC1 DeInit */
    ADC_DeInit(ADC1);
    /* Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
    /* Initialize ADC structure */
    ADC_StructInit(&ADC_InitStructure);
    /* Configure the ADC1 in continous mode with a resolutuion equal to 12 bits  */
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
    ADC_Init(ADC1,&ADC_InitStructure);
    // channels selection and it's sampling time config
    ADC_ChannelConfig(ADC1,ADC_Channel_0,ADC_SampleTime_55_5Cycles);
    ADC_ChannelConfig(ADC1,ADC_Channel_4,ADC_SampleTime_55_5Cycles);
    /* ADC Calibration */
    ADC_GetCalibrationFactor(ADC1);
    /* ADC DMA request in circular mode */
    //ADC_DMARequestModeConfig(ADC1,ADC_DMAMode_Circular);
    ADC_DMACmd(ADC1,ENABLE);
    /* Enable ADC1 */
    ADC_Cmd(ADC1,ENABLE);
    while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_ADEN));/* Wait the ADCEN falg */
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)); 
    /* ADC1 regular Software Start Conv */
    ADC_StartOfConversion(ADC1);
}


//*****************************************************************************
//Инициализация вентиляторов конденсатора и испарителя в режиме ШИМ
//*****************************************************************************
void initializeFans(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef timerInitStructure;
    TIM_OCInitTypeDef PWMChannelInit;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN, ENABLE);
	
// FAN конденсатора
	// Init port_B
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;	
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_1);
	
    // Init TIM3
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    // Запускаем таймер на тактовой частоте в 12 mHz
    timerInitStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 12000000) - 1;
    // Период => TIM_Clock/TIM_Period = frequency PWM
    timerInitStructure.TIM_Period = 2000-1; //(6kHz)
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &timerInitStructure);
    TIM_Cmd(TIM3, ENABLE);
    
    PWMChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    PWMChannelInit.TIM_Pulse = 0;   //duty
    PWMChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    PWMChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &PWMChannelInit);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

// FAN испарителя
	// Init port_B
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;	
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_1);
	
    PWMChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    PWMChannelInit.TIM_Pulse = 0;   //duty
    PWMChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    PWMChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM3, &PWMChannelInit);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

//*****************************************************************************
//Инициализация TACHO Condenser, TACHO Evaporator
//*****************************************************************************
//TIM16_CCR1  0x40014400 + 0x34
void initializeTachoFans(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;	//TACHO Condenser
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;		//TACHO Evaporator
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_2);
    
    //Настройка DMA для TACHO Evaporator (2-й канал DMA)
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;                       // включить тактирование DMA1
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;                   // включить тактирование SYSCFG
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_TIM17_DMA_RMP;            // переключить запрос TIM17 на канал 2
    DMA1_Channel2->CPAR = (uint32_t)&(TIM17->CCR1);         // адрес перифериии
    DMA1_Channel2->CMAR = (uint32_t)&FanEvaporatorStr.buffTacho[0]; // адрес памяти
    DMA1_Channel2->CNDTR = 2;                               // количество пересылаемых значений
    DMA1_Channel2->CCR &= ~DMA_CCR_MEM2MEM;                 // не из памяти в память
    DMA1_Channel2->CCR &= ~DMA_CCR_PL;                      // приоритет низкий
    DMA1_Channel2->CCR |= DMA_CCR_PL_1;                     // приоритет высокий
    DMA1_Channel2->CCR |= DMA_CCR_MSIZE_0;                  // размер данных 16 бит
    DMA1_Channel2->CCR |= DMA_CCR_PSIZE_0;                  // размер периферии 16 бит
    DMA1_Channel2->CCR |= DMA_CCR_MINC;                     // память инкрементировать
    DMA1_Channel2->CCR &= ~DMA_CCR_PINC;                    // периферию не инкрементировать
//    DMA1_Channel2->CCR &= ~DMA_CCR_CIRC;                    // циркулярный режим выключен
    DMA1_Channel2->CCR |= DMA_CCR_CIRC;                     // циркулярный режим включен
    DMA1_Channel2->CCR &= ~DMA_CCR_DIR;                     // направление из периферии в память
//    DMA1_Channel2->CCR |= DMA_CCR_TCIE;                     // прерывание по завершению транзакции вкл.
    DMA1_Channel2->CCR &= ~DMA_CCR_TCIE;                    // прерывание по завершению транзакции выкл.
    DMA1_Channel2->CCR |= DMA_CCR_EN;                       // Разрешаем работу 2-го канала DMA
    
    //Настройка DMA для TACHO Condenser (3-й канал DMA)
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;                       // включить тактирование DMA1
//    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;                   // включить тактирование SYSCFG
//    SYSCFG->CFGR1 |= SYSCFG_CFGR1_TIM16_DMA_RMP;            // переключить запрос TIM16 на канал 4
    DMA1_Channel3->CPAR = (uint32_t)&(TIM16->CCR1);         // адрес перифериии
    DMA1_Channel3->CMAR = (uint32_t)&FanCondenserStr.buffTacho[0];  // адрес памяти
    DMA1_Channel3->CNDTR = 2;                               // количество пересылаемых значений
    DMA1_Channel3->CCR &= ~DMA_CCR_MEM2MEM;                 // не из памяти в память
    DMA1_Channel3->CCR &= ~DMA_CCR_PL;                      // приоритет низкий
    DMA1_Channel3->CCR |= DMA_CCR_PL_1;                     // приоритет высокий
    DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0;                  // размер данных 16 бит
    DMA1_Channel3->CCR |= DMA_CCR_PSIZE_0;                  // размер периферии 16 бит
    DMA1_Channel3->CCR |= DMA_CCR_MINC;                     // память инкрементировать
    DMA1_Channel3->CCR &= ~DMA_CCR_PINC;                    // периферию не инкрементировать
//    DMA1_Channel3->CCR &= ~DMA_CCR_CIRC;                    // циркулярный режим выключен
    DMA1_Channel3->CCR |= DMA_CCR_CIRC;                    // циркулярный режим включен
    DMA1_Channel3->CCR &= ~DMA_CCR_DIR;                     // направление из периферии в память
//    DMA1_Channel3->CCR |= DMA_CCR_TCIE;                     // прерывание по завершению транзакции вкл.
    DMA1_Channel3->CCR &= ~DMA_CCR_TCIE;                     // прерывание по завершению транзакции выкл.
    DMA1_Channel3->CCR |= DMA_CCR_EN;                       // Разрешаем работу 3-го канала DMA
    
//    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1);
//    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);


    // Init TIM16 for TACHO1
    /*Инициализация таймера TIM16.
    Для измерения периода входного сигнала используется канал 1 (TIM16_CH1)*/
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;    //Включаем тактирование TIM16
    TIM16->PSC = (uint16_t)(SystemCoreClock/10000)-1;  //новая частота 10Khz
    TIM16->CCMR1 |= TIM_CCMR1_CC1S_0;       //Выбор активного входа. Записываем "01" в биты CC1S - связываем регистр TIM16_CCR1 со входом TI1 
    TIM16->CCER &= ~TIM_CCER_CC1P;          //По переднему фронту - положительный перепад импульса
    TIM16->CCMR1 &= ~TIM_CCMR1_IC1PSC;      //Предделитель таймера отключен
	TIM16->CCMR1 |= TIM_ICPSC_DIV2;         //Предделитель входного сигнала /2
    TIM16->CCER |= TIM_CCER_CC1E;           //Разрешен захват значения счетчика в регистр TIM16_CCR1
    //TIM16->DIER |= TIM_DIER_CC1IE;        //Разрешена генерация прерывания при захвате
    TIM16->DIER |= TIM_DIER_CC1DE;          //разрешаем формировать запрос к DMA
    TIM16->CR1 |= TIM_CR1_CEN;              //Запускаем счет таймера
    
    
// Init TIM17 for TACHO2
    /*Инициализация таймера TIM17.
    Для измерения периода входного сигнала используется канал 1 (TIM17_CH1)*/
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    //Включаем тактирование TIM17
    TIM17->PSC = (uint16_t)(SystemCoreClock/10000)-1;  //новая частота 10Khz
    TIM17->CCMR1 |= TIM_CCMR1_CC1S_0;       //Выбор активного входа. Записываем "01" в биты CC1S - связываем регистр TIM17_CCR1 со входом TI1 
    TIM17->CCER &= ~TIM_CCER_CC1P;          //По переднему фронту - положительный перепад импульса
    TIM17->CCMR1 &= ~TIM_CCMR1_IC1PSC;      //Предделитель таймера отключен
	TIM17->CCMR1 |= TIM_ICPSC_DIV2;         //Предделитель входного сигнала /2
    TIM17->CCER |= TIM_CCER_CC1E;           //Разрешен захват значения счетчика в регистр TIM17_CCR1
    //TIM17->DIER |= TIM_DIER_CC1IE;        //Разрешена генерация прерывания при захвате
    TIM17->DIER |= TIM_DIER_CC1DE;          //разрешаем формировать запрос к DMA
    TIM17->CR1 |= TIM_CR1_CEN;              //Запускаем счет таймера
}


//*****************************************************************************
//Инициализация реле
//*****************************************************************************
void initializeRelays(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
#ifdef GP_RELAY_HEAT_BOX
    pin_OUT_PP(GP_RELAY_HEAT_BOX);
#endif
    pin_OUT_PP(GP_RELAY_ALARM);
    pin_OUT_PP(GP_RELAY_FAN_H2);
}


//*****************************************************************************
//Инициализация светодиодов
//*****************************************************************************
void initializeLeds(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    pin_OUT_OD(GP_LED_STAT);
    GPIO_SetBits(GP_LED_STAT);
    pin_OUT_OD(GP_LED_ERR);
    GPIO_SetBits(GP_LED_ERR);
}


//*****************************************************************************
//Инициализация датчиков температуры и первый запуск измерения температур
//*****************************************************************************
void initializeSensorsTemp(void)
{
#ifdef GP_T1
    one_wire_ini_setting(GP_T1);
    one_ware_convert_t(GP_T1);
#endif
#ifdef GP_T3
    one_wire_ini_setting(GP_T3);
    one_ware_convert_t(GP_T3);
#endif
#ifdef GP_T4
    one_wire_ini_setting(GP_T4);
    one_ware_convert_t(GP_T4);
#endif
#ifdef GP_T6
    one_wire_ini_setting(GP_T6);
    one_ware_convert_t(GP_T6);
#endif
#ifdef GP_T7
    one_wire_ini_setting(GP_T7);
    one_ware_convert_t(GP_T7);
#endif
#ifdef GP_T8
    one_wire_ini_setting(GP_T8);
    one_ware_convert_t(GP_T8);
#endif
    computeTempVal();
}


//********************************************************************************
//Инициализация USART1 для ВНУТРЕННЕГО ModBus
//********************************************************************************
void initializeUSART1ModBusInt(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* Enable GPIOB clock and USART2*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* Configure USART1 RX (PB7) as alternate function push-pull*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_0);

    /* Configure USART1 TX (PB6) as alternate function push-pull*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_0);

    /* Configure PB3 as rs485 DE select*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //setting parametrs common for all uarts
    USART_InitStructure.USART_BaudRate            = 9600;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //включаем прерывание по приёму
    USART_Cmd(USART1, ENABLE);                      //включаем UART

    NVIC_SetPriority(USART1_IRQn, 0); 	            //задаем приоритет прерывания
    NVIC_EnableIRQ(USART1_IRQn); 				    //разрешаем прерывания
}


//********************************************************************************
//Инициализация USART2 для ВНЕШНЕГО ModBus
//********************************************************************************
void initializeUSART2ModBusExt(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* Enable GPIOA clock and USART2*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* Configure USART2 RX (PA3) as alternate function push-pull*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

    /* Configure USART2 TX (PA2) as alternate function push-pull*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);

    /* Configure PA1 as rs485 DE select*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //setting parametrs common for all uarts
    USART_InitStructure.USART_BaudRate            = numSpeed[W_SPEED];
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  //включаем прерывание по приёму
    USART_Cmd(USART2, ENABLE);

    NVIC_SetPriority(USART2_IRQn, 1);               //задаем приоритет прерывания
    NVIC_EnableIRQ(USART2_IRQn); 				    //разрешение прерывания
}


//********************************************************************************
//Инициализация таймера для отчёта времени окончания приёма данных по ModBus
//********************************************************************************
void initializeTimerModBusTimeout(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 , ENABLE);
    TIM_DeInit(TIM14);

    //0.0001 sec setup APB=48Mhz/(48*100)
    TIM_TimeBaseStructure.TIM_Prescaler         = 48-1;                 //Настраиваем на мкс
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = 100;                  //Считаем до 100 мкс
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

    TIM_ClearFlag(TIM14, TIM_FLAG_Update);
    TIM_ITConfig(TIM14,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM14, ENABLE);

    NVIC_SetPriority(TIM14_IRQn, 2);                //задаем приоритет прерывания
    NVIC_EnableIRQ(TIM14_IRQn);                     //разрешение прерывания
}



//********************************************************************************
//Инициализация I2C для связи с платой компрессора
//********************************************************************************
//void I2C1_init(void)
//{
//    I2C_InitTypeDef  I2C_InitStructure;
//    GPIO_InitTypeDef  GPIO_InitStructure;
// 
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
// 
//    /* Configure I2C_EE pins: SCL and SDA */
//    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
// 
//    /* I2C configuration */
//    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
//    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//    I2C_InitStructure.I2C_ClockSpeed = 100000;
// 
//    /* I2C Peripheral Enable */
//    I2C_Cmd(I2C1, ENABLE);
//    /* Apply I2C configuration after enabling it */
//    I2C_Init(I2C1, &I2C_InitStructure);
//}


