/* Includes ------------------------------------------------------------------*/
#include "f_air_control.h"


void funcRunDelayed(uint8_t timerNum, uint32_t setTime)
{
    if (!TimersStr.flagCountEnable[timerNum]) {
//        TimersStr.counter[timerNum] = 0;
        TimersStr.flagCountEnable[timerNum] = ENABLE;
    } else {
        if (TimersStr.counter[timerNum] >= setTime) {
            switch (timerNum) {
                case NT_TEMP_VAL:
                    computeTempVal();
                    break;
                case NT_RPM_FANI:
                    computeRpmFanI();
                    break;
                case NT_RPM_FANC:
                    computeRpmFanC();
                    break;
                case NT_RPM_COMPR:
                    computeRpmCompress();
                    break;
                case NT_LED_ALARM:
                    blinkLedAlarm();
                    break;
                case NT_LED_STAT:
                    blinkLedStat();
                    break;
                case NT_CHANGE_MODB:
                    changeModbusAddressSpeed();
                    break;
                case NT_CONTROL_FANC:
                    updateSpeedFAN_C();
                    break;
                case NT_CONTROL_FANI:
                    updateSpeedFAN_I();
                    break;
                case NT_RUN_1S:
                    runOnce1s();
                    break;
                case NT_UPDATE_COMPRESS_STATE:
                    updateCompressorState();
                    break;
                case NT_CHECK_ERR:
                    checkError();
                default:
                    break;
            }
            TimersStr.counter[timerNum] = 0;
            TimersStr.flagCountEnable[timerNum] = DISABLE;
        }   
    }
}



//********************************************************************************
//Функции для получения значений температур и ошибок
//Функция возвращает состояние аварий датчиков (0 - нет аварий, 1- есть как минимум одна авария)
uint8_t getErrorSensor(int16_t valueSensor)
{
    if (valueSensor & 0x8000){
        return ~valueSensor & 0x7000 ? 1 : 0;
    } else {
        return valueSensor & 0x7000 ? 1: 0;
    }
}

//Функция возвращает округлённое значение температуры без регистров аварий
int16_t getValueSensor(int16_t valueSensor)
{
    if (valueSensor & 0x8000){          //если значение отрицательное
        return (valueSensor | 0xF000)/10;    //возвращаем отрицательное значение без рег. аварий
    } else {                            //иначе если значение положительное
        return (valueSensor & 0x0FFF)/10;    //возвращаем положительное значение без рег. аварий
    }
}


//********************************************************************************
//Функция с задержкой выполнения 1 раз в секунду
//********************************************************************************
void runOnce1s (void)
{
    static uint32_t timerOnFanH2 = 0;
    // если FAN водорода выключен
    if (!STATE_FAN_H2){
      timerOnFanH2++;
      // если прошло 6 часов
      if(timerOnFanH2 >= 21600) {
        GPIO_SetBits(GP_RELAY_FAN_H2);          //включаем вентилятор водорода
        R_STATES |= (1<<9);
        timerOnFanH2 = 0;
      }
    } else {
      timerOnFanH2++;
      // если прошло 5 мин.
      if(timerOnFanH2 >= 300) {
        GPIO_ResetBits(GP_RELAY_FAN_H2);        //выключаем вентилятор водорода
        R_STATES &=~(1<<9);
        timerOnFanH2 = 0;
      }
    }
    
// **************** операции для компрессора *************
    //если компрессор не "упавший"
    //и есть команда на запуск
    //и текущие обороты компрессора больше чем задан. +100
    if(!CompresStr.flagFault
    && CompresStr.commandStartStop == C_START
    && (R_RPM_COMPRES > (W_MAX_RPM_COMPRESS + 100))) {
      if(DUTY_SPEED_COMPRES < MAX_DUTY_COMPRES) {
          DUTY_SPEED_COMPRES = DUTY_SPEED_COMPRES + 1;    // уменьшаем обороты (инверсное задание)
      }
    }

    if(CompresStr.commandStartStop == C_START 
    && !CompresStr.flagSetStart
    && CompresStr.flagFault == 0) {
        CompresStr.countTimeStart++;
    } else {
        CompresStr.countTimeStart = 0;
    }
    
    if(CompresStr.commandStartStop == C_START
    && CompresStr.flagSetStart 
    && CompresStr.flagFault == 0) {
        CompresStr.countTimeWork++;
    } else {
        CompresStr.countTimeWork = 0;
    }
    
    if(CompresStr.flagFault) {
        CompresStr.countTimeRestart++;
    } else {
        CompresStr.countTimeRestart = 0;
    }
    
    
    // увеличиваем таймер компрессора
    if (CompresStr.timerMain < 0xFFFE) {
        CompresStr.timerMain++;
    }
    //таймеры состояния компрессора
    if (!STATE_COMPRESSOR) {
        if (CompresStr.timerMain < 0xFFFE) {
            CompresStr.timerOff++;
        }
        CompresStr.timerOn = 0;
    } else {
        if (CompresStr.timerMain < 0xFFFE) {
            CompresStr.timerOn++;
        }
        CompresStr.timerOff = 0;
    }
    // увеличиваем счётчик задержки на отключение компрессора
    CompresStr.flagConfirm ? CompresStr.countDelayOff++ : 0;
    // увеличиваем таймер вентилятора конденсатора
    FanCondenserStr.timerStart < DEF_DELAY_START_FAN_COND ? FanCondenserStr.timerStart++ : DEF_DELAY_START_FAN_COND;
    // увеличиваем таймер проверки состояния вентилятора испарителя
// **************** операции для компрессора END*************    
    
    //  FanEvaporatorStr.statChecking ? FanEvaporatorStr.timerCheck++ : 0;
    modBusExt.countReqTest++;   //для выхода из режима "тест" в случае обрыва связи
    
}

//********************************************************************************
//Функция обновляет состояние компрессора
//********************************************************************************
void updateCompressorState (void)
{
    static uint16_t muxTimeFault = 1;
    //инверсное управление PWM
    if (CompresStr.commandStartStop == C_START) {
        // если не установлено стартовое значение и компрессор не "падал"
        if(!CompresStr.flagSetStart && !CompresStr.flagFault) {
            DUTY_SPEED_COMPRES = 750;               // установка начального значения задания оборотов
            if(CompresStr.countTimeStart < 3) {     // если не прошло 3с
                CompresStr.flagSetStart = 1;
            }
        } else {
            if(!CompresStr.flagFault) {                           // если компрессор не "падал"
                if(CompresStr.countTimeWork < 40) {		            // если не прошло 40с
                    if(R_RPM_COMPRES < W_MAX_RPM_COMPRESS) {	    // если обороты не достигли заданного значения 
                        // увеличиваем обороты (инверсное задание)
                        if(DUTY_SPEED_COMPRES > 0) {
                           DUTY_SPEED_COMPRES = DUTY_SPEED_COMPRES - 1;
                        }
                    } else {
//                    if(R_RPM_COMPRES > W_MAX_RPM_COMPRESS + 100) { //если текущие обороты компрессора больше чем задан. +100
//                        if(DUTY_SPEED_COMPRES < MAX_DUTY_COMPRES) {
//                            DUTY_SPEED_COMPRES = DUTY_SPEED_COMPRES + 1;    // уменьшаем обороты (инверсное задание)
//                        }
//                    }
                        CompresStr.countTimeWork = 0;
                        CompresStr.countFault = 0;
                    }
                } else {
                    //иначе если таймер переполнился и обороты не достигли заданного значения
                    CompresStr.flagFault = 1;
                    CompresStr.flagSetStart = 0;
                    DUTY_SPEED_COMPRES = MAX_DUTY_COMPRES;  //останавливаем компрессор
                }
            }
        }
        
        // если время (в сек.) для рестарта вышло (время инкрементмруется в функции "runOnce1s")
        if(CompresStr.countTimeRestart >= 30 * muxTimeFault) {
            // если было 3 неудачных старта
            if(CompresStr.countFault >= 3) {
                muxTimeFault++;     // увеличиваем множитель времени паузы
            }
            CompresStr.countFault++;
            CompresStr.flagFault = 0;
            CompresStr.countTimeStart = 0;
            CompresStr.countTimeWork = 0;
            CompresStr.countTimeRestart = 0;
            CompresStr.flagSetStart = 0;
        }
        
        // если заданная мощность > 40% и обороты < 100RPM и ток = 0
        if(R_COMPRESS > 40 && R_RPM_COMPRES <= 100 && R_CURRENT_AMP == 0) {
            DUTY_SPEED_COMPRES = MAX_DUTY_COMPRES;  // выключаем компрессор
            CompresStr.flagFault = 1;               // выставляем флаг "компрессор упал"
        }

    } else if (CompresStr.commandStartStop == C_STOP){
        DUTY_SPEED_COMPRES = MAX_DUTY_COMPRES;
        CompresStr.countFault = 0;
        CompresStr.flagFault = 0;
        CompresStr.countTimeStart = 0;
        CompresStr.countTimeWork = 0;
        CompresStr.countTimeRestart = 0;
        CompresStr.flagSetStart = 0;
    }
    R_COMPRESS = (uint16_t)((MAX_DUTY_COMPRES - DUTY_SPEED_COMPRES) / (MAX_DUTY_COMPRES/100));   //записываем скорость компрессора (%) в карту
}


//********************************************************************************
//Регулирование скорости вращения FAN конденсатора
//********************************************************************************
void updateSpeedFAN_C (void)
{
    static int16_t pastValueT7;
    static uint8_t changeValueT7;
    // проверка на увеличение или уменьшение температуры на t7
    if (BUF_T7 != pastValueT7 
    && !getErrorSensor(BUF_T7)) {
        if (BUF_T7 > pastValueT7) {
            changeValueT7 = 1;  //температура растёт
        } else {
            changeValueT7 = 0;  //температура падает
        }
        pastValueT7 = BUF_T7;
    }
    //Обновление значения скорости вращения вентелятора конденсатора
    if (!BIT_TEST) {
        if (changeValueT7 && (getValueSensor(BUF_T7) >= W_TZPK)) {
            //увеличиваем скорость вращения вент. конденсатора
            if (DUTY_SPEED_FAN_C >= MAX_DUTY_SPEED_FAN) {
                DUTY_SPEED_FAN_C = MAX_DUTY_SPEED_FAN;
            } else {
                DUTY_SPEED_FAN_C++;
            }
        }
        if (!changeValueT7 && (getValueSensor(BUF_T7) < W_TZMK)){
            //уменьшаем скорость вращения вент. конденсатора
            if (DUTY_SPEED_FAN_C <= MIN_DUTY_SPEED_FAN) {
                DUTY_SPEED_FAN_C = MIN_DUTY_SPEED_FAN;
            } else {
                DUTY_SPEED_FAN_C--;
            }
        }
    }
}


//********************************************************************************
//Регулирование скорости вращения FAN испарителя
//********************************************************************************
void updateSpeedFAN_I (void)
{
  if (DUTY_SPEED_FAN_I < MIN_DUTY_SPEED_FAN && STATE_COMPRESSOR) {
    DUTY_SPEED_FAN_I = (uint16_t)(((uint32_t) 40 * MAX_DUTY_SPEED_FAN)/100);    //скорость вент. на 40%
  }
  //увеличиваем обороты FAN испарителя
  DUTY_SPEED_FAN_I = DUTY_SPEED_FAN_I < MAX_DUTY_SPEED_FAN ? DUTY_SPEED_FAN_I++ : DUTY_SPEED_FAN_I;
}


//********************************************************************************
//Изменение адреса и скорости для внешнего ModBus
//********************************************************************************
void changeModbusAddressSpeed (void)
{
    modBusExt.adress = W_ADDRESS;					//Обновление ModBus адреса ВНЕШНЕГО
    //Обновление скорости USART для ВНЕШНЕГО
    modBusExt.speed = W_SPEED;
    USART2->CR1 &= (uint32_t)~((uint32_t)USART_CR1_UE);
    switch(modBusExt.speed){
        case 0:
            modBusExt.delay = 150;
            break;
        case 1:
            modBusExt.delay = 75;
            break;
        case 2:
            modBusExt.delay = 48;
            break;
        case 3:
            modBusExt.delay = 38;
            break;
        case 4:
            modBusExt.delay = 25;
            break;
        case 5:
            modBusExt.delay = 19;
            break;
        default:
            modBusExt.delay = 20;
            break;
        }
    initializeUSART2ModBusExt();
}


//********************************************************************************
//Переключение состояния светодиода статуса в режиме ТЕСТ
//********************************************************************************
void blinkLedStat (void)
{
    if (BIT_TEST) {
        GPIO_ToggleBits(GP_LED_STAT);   //переключаем светодиод 
    } else {
        GPIO_ResetBits(GP_LED_STAT);
    }
}

//********************************************************************************
//Переключение состояния светодиода аварии и реле
//********************************************************************************
void blinkLedAlarm (void)
{
    if (R_ALARM) {
        GPIO_ToggleBits(GP_LED_ERR);
        GPIO_SetBits(GP_RELAY_ALARM);   //relay alarm ON
    } else {
        GPIO_SetBits(GP_LED_ERR);
        GPIO_ResetBits(GP_RELAY_ALARM); //relay alarm OFF
    }
}


//********************************************************************************
//Определение скорости вращения компрессора
//********************************************************************************
void computeRpmCompress (void)
{
    //если скорость слишком низкая
    if (TIM15->SR & TIM_SR_UIF) {
#ifndef DEBUG_COMPRESS
        TIM15->SR &= ~TIM_SR_UIF;
        TIM15->CNT = 0;
        CompresStr.countTacho = 0;
        CompresStr.speedRPM = 0;
#endif
    } else {
#ifndef DEBUG_COMPRESS
        if (CompresStr.buffCheckStop == CompresStr.countTacho) {
            if (CompresStr.countCheckStop > 3) {
                CompresStr.timePeriodTurn = 0;
                CompresStr.countTacho = 0;
                CompresStr.speedRPM = 0;
								CompresStr.countCheckStop = 0;
                TIM15->CNT = 0;
            } else {
                CompresStr.countCheckStop++;
            }
        } else {
					CompresStr.countCheckStop = 0;
		}
				
        CompresStr.speedRPM = ((1000000 / CompresStr.timePeriodTurn) * 6);
#endif
        if (CompresStr.speedRPM > 0x8000) {
            R_RPM_COMPRES = -1;
        } else if (CompresStr.speedRPM < 100) {
            R_RPM_COMPRES = 0;
        } else {
            R_RPM_COMPRES = CompresStr.speedRPM;
        }
    }
    CompresStr.buffCheckStop = CompresStr.countTacho;
}


//********************************************************************************
//Определение скорости вращения вентилятора конденсатора
//********************************************************************************
void computeRpmFanC (void)
{
    //FAN C
    if (DMA1->ISR & DMA_ISR_TCIF3) {
        DMA1_Channel3->CCR &= ~DMA_CCR_EN;
        TIM16->EGR |= TIM_EGR_UG;           // сбрасываем счётчики
        DMA1->IFCR |= DMA_IFCR_CGIF3;       // сбрасываем все флаги прерывания DMA
        //Чтение скорости вентилятора TACHO2
        if (FanCondenserStr.buffTacho[1] > FanCondenserStr.buffTacho[0]){
            FanCondenserStr.period = FanCondenserStr.buffTacho[1] - FanCondenserStr.buffTacho[0];
        }
        DMA1_Channel3->CCR |= DMA_CCR_EN;
    }
    //Для определения остановки вентилятора конденсатора
    if (FanCondenserStr.buffCheckStop == FanCondenserStr.buffTacho[1]){
        if (FanCondenserStr.countCheckStop > 5){
            FanCondenserStr.period = 0;
            FanCondenserStr.buffTacho[0] = 0;
            FanCondenserStr.buffTacho[1] = 0;
            FanCondenserStr.countCheckStop = 0;
        } else {
            FanCondenserStr.countCheckStop++;
        }
    } else {
        FanCondenserStr.countCheckStop = 0;
    }
    FanCondenserStr.buffCheckStop = FanCondenserStr.buffTacho[1];
#ifndef DEBUG_FANS
    R_RPM_FAN_C = (uint16_t)(((1000000 / FanCondenserStr.period) * 60)/100);
#endif
}


//********************************************************************************
//Определение скорости вращения вентилятора испарителя
//********************************************************************************
void computeRpmFanI (void)
{
    //FAN I
    if (DMA1->ISR & DMA_ISR_TCIF2){
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        TIM17->EGR |= TIM_EGR_UG;           // сбрасываем счётчики
        DMA1->IFCR |= DMA_IFCR_CGIF2;       // сбрасываем все флаги прерывания DMA
        //Чтение скорости вентилятора TACHO1
        if (FanEvaporatorStr.buffTacho[1] > FanEvaporatorStr.buffTacho[0]){
            FanEvaporatorStr.period = FanEvaporatorStr.buffTacho[1] - FanEvaporatorStr.buffTacho[0];
        }
        DMA1_Channel2->CCR |= DMA_CCR_EN;
    }
    //Для определения остановки вентилятора испарителя
    if (FanEvaporatorStr.buffCheckStop == FanEvaporatorStr.buffTacho[1]){
        if (FanEvaporatorStr.countCheckStop > 5){
            FanEvaporatorStr.period = 0;
            FanEvaporatorStr.buffTacho[0] = 0;
            FanEvaporatorStr.buffTacho[1] = 0;
            FanEvaporatorStr.countCheckStop = 0;
        } else {
            FanEvaporatorStr.countCheckStop++;
        }
    } else {
        FanEvaporatorStr.countCheckStop = 0;
    }
    FanEvaporatorStr.buffCheckStop = FanEvaporatorStr.buffTacho[1];
#ifndef DEBUG_FANS
    R_RPM_FAN_I = (uint16_t)(((1000000 / FanEvaporatorStr.period) * 60)/100);
#endif
}




//********************************************************************************
//обновление всех измеренных температур и запуск новых измерений
//********************************************************************************
void computeTempVal(void)
{
    uint16_t temp;
/************************************************************/
#ifdef GP_T1
    temp = one_wire_read_byte_dec(GP_T1);       //температура окружающая
    if((temp == 850) && TempStr.count85C[1] < 20) {
        count85C[1]++;
    } else {
        TempStr.count85C[1] = 0;
    #ifndef DEBUG_TEMPERATURE
        if ((temp == ERROR_SC_GND || temp == ERROR_NO_SENSOR || temp == ERROR_CRC)
        && TempStr.countErr[1] < SENSOR_COUNT_ERR) {
            TempStr.countErr[1]++;
        } else {
            BUF_T1 = temp;
            TempStr.countErr[1] = 0;
        }
    #endif
        R_T1 = convertTemperature(BUF_T1);
    }
#endif
/************************************************************/
#ifdef GP_T3
    temp = one_wire_read_byte_dec(GP_T3);       //температура на входе (внутренняя)
    if((temp == 850) && TempStr.count85C[3] < 20) {
        TempStr.count85C[3]++;
    } else {
        TempStr.count85C[3] = 0;
    #ifndef DEBUG_TEMPERATURE
        if ((temp == ERROR_SC_GND || temp == ERROR_NO_SENSOR || temp == ERROR_CRC)
        && TempStr.countErr[3] < SENSOR_COUNT_ERR) {
            TempStr.countErr[3]++;
        } else {
            BUF_T3 = temp;
            TempStr.countErr[3] = 0;
        }
    #endif
        R_T3 = convertTemperature(BUF_T3);
    }
#endif
/************************************************************/
#ifdef GP_T6
    temp = one_wire_read_byte_dec(GP_T6);       //температура на всасывающей трубе испарителя
    if((temp == 850) && TempStr.count85C[6] < 20){
        TempStr.count85C[6]++;
    } else {
        TempStr.count85C[6] = 0;
    #ifndef DEBUG_TEMPERATURE
        if ((temp == ERROR_SC_GND || temp == ERROR_NO_SENSOR || temp == ERROR_CRC)
        && TempStr.countErr[6] < SENSOR_COUNT_ERR) {
            TempStr.countErr[6]++;
        } else {
            BUF_T6 = temp;
            TempStr.countErr[6] = 0;
        }
    #endif
        R_T6 = convertTemperature(BUF_T6);
    }
#endif
/************************************************************/
#ifdef GP_T7
    temp = one_wire_read_byte_dec(GP_T7);       //температура на сливе конденсатора
    if((temp == 850) && TempStr.count85C[7] < 20){
        TempStr.count85C[7]++;
    } else {
        TempStr.count85C[7] = 0;
    #ifndef DEBUG_TEMPERATURE
        if ((temp == ERROR_SC_GND || temp == ERROR_NO_SENSOR || temp == ERROR_CRC)
        && TempStr.countErr[7] < SENSOR_COUNT_ERR) {
            TempStr.countErr[7]++;
        } else {
            BUF_T7 = temp;
            TempStr.countErr[7] = 0;
        }
    #endif
        R_T7 = convertTemperature(BUF_T7);
    }
#endif
/************************************************************/	
#ifdef GP_T8
    temp = one_wire_read_byte_dec(GP_T8);       //температура картера
    if((temp == 850) && TempStr.count85C[8] < 20){
        TempStr.count85C[8]++;
    } else {
        TempStr.count85C[8] = 0;
    #ifndef DEBUG_TEMPERATURE
        if ((temp == ERROR_SC_GND || temp == ERROR_NO_SENSOR || temp == ERROR_CRC)
        && TempStr.countErr[8] < SENSOR_COUNT_ERR) {
            TempStr.countErr[8]++;
        } else {
            BUF_T8 = temp;
            TempStr.countErr[8] = 0;
        }
    #endif
        R_T8 = convertTemperature(BUF_T8);
    }
#endif
/************************************************************/	
//Запуск измерения температур
#ifdef GP_T1
    one_ware_convert_t(GP_T1);
#endif
#ifdef GP_T3
    one_ware_convert_t(GP_T3);
#endif
#ifdef GP_T6
    one_ware_convert_t(GP_T6);
#endif
#ifdef GP_T7
    one_ware_convert_t(GP_T7);
#endif
#ifdef GP_T8
    one_ware_convert_t(GP_T8);
#endif
}
    
// дополнителяная фун-циф для преобразования значений темпертур в "старый формат"
uint16_t convertTemperature(uint16_t value)
{
  if (value & 0x8000) {
      if (value == ERROR_NO_SENSOR) return 0x100;
      else if (value == ERROR_SC_GND) return 0x200;
      else if (value == ERROR_CRC) return 0x400;
      else return (((~value + 1) & 0xFFF) / 10) | 0x80;
  } else {
      return ((value & 0xFFF) / 10);
  }         
}


//********************************************************************************
//вычисление значения напряжения питания и тока компрессора
//********************************************************************************
void computeValueADC (void)
{
  /* Читаем значение АЦП и усредняем*/
  if(DMA_GetCurrDataCounter(DMA1_Channel1) == 1){
    uint8_t i = 0;
    uint32_t adcSumV = 0;
    uint32_t adcSumC = 0;
    
    while(i < 40)
    {
        adcSumV = ADCStr.buffData[i+1] + adcSumV;
        adcSumC = ADCStr.buffData[i] + adcSumC;
        i = i+2;
    }
    ADCStr.voltage = adcSumV/20;
    ADCStr.current = adcSumC/20;
    
    R_POWER_VOLT = (uint16_t)(adcSumV/20)*1.55;
    
    float voltRatio = 3.6/4095;		//0.000879    коэффициент перевода имерений в напряжение
//				uint8_t MaxCurrent = 20;
//				float kVolt = (3.6/2 - 3.6*0.1) / MaxCurrent; 
    float CurrentPinVoltage = voltRatio * (adcSumC/20); 
    float CurrentValue = ((2.5 - CurrentPinVoltage)/1.3) *1000;
//				float CurrentValue = (2.53 - CurrentPinVoltage)*1000;
    R_CURRENT_AMP = (uint16_t)CurrentValue;
    
    R_POWER = (uint16_t)((R_POWER_VOLT * R_CURRENT_AMP) / 10000); 
  }
  ADC1->CR |= ADC_CR_ADSTART;             // измеряем напряжение         
}



//*****************************************************************************
//Управление компрессора
//*****************************************************************************
void controlCompress(void)
{
    //если перегрев компрессора
    if (ALARM_COMPRES_FAULTY){										
        CompresStr.flagConfirm = 0;              //включение компрессора запрещено
    } else {
        //если температура ккомпрессора >= W_MIN_T_COMPRES
        //и нет обмерзания испарителя
        //и нет аварии по высокому давлению
        //и нет аварии по низкой температуре
        //и нет аварии по питанию +48В
        //и нет ошибок датчиков температуры
        //и нет аварии вентилятора испарителя
        //и нет аварии вентилятора конденсатора
        if ((getValueSensor(BUF_T8) >= W_MIN_T_COMPRES)
        && !(ALARM_FREEZ_EVAP)
        && !ALARM_HI_PRESS
        && !ALARM_LO_INT_TEMP
        && !ALARM_48V
        && !getErrorSensor(BUF_T7)
        && !getErrorSensor(BUF_T8)
        && !getErrorSensor(BUF_T3)
        && !getErrorSensor(BUF_T6)
        && !ALARM_FAN_I
        && !ALARM_FAN_C){
            CompresStr.flagConfirm = 1;              //включение компрессора разрешено
            CompresStr.countDelayOff = 0;
        } else {
            //если счётчик ошибок меньше 4
            if (CompresStr.countDelayOff < 4){
                return;
            } else {
                CompresStr.flagConfirm = 0;	        //включение компрессора запрещено
            }
        }
    }
    //если включение разрешено
    if (CompresStr.flagConfirm){
        switch(CompresStr.stepNum){
            //первоначальное включение компрессора
            case FIRST_ON:
                //проверка состояния задержки на включение
                if ((CompresStr.timerMain <= DEF_3MIN) 
                    && (getValueSensor(BUF_T3) >= W_T_INT)){
                      R_STATES |= (1<<8);             //состояние задержки на включение ON
                } else {
                    R_STATES &=~ (1<<8);
                }
                //если прошло 3 минуты и t внутр. >= t заданной
                //или если t внутр. >= t(a1) срочного включения
                //и если прошла обязательная задержка
                if ((((CompresStr.timerMain >= DEF_3MIN) && (getValueSensor(BUF_T3) >= (W_T_INT)))
                || (getValueSensor(BUF_T3) >= W_TQON))
                && CompresStr.timerOff >= DEF_1MIN){
                    CompresStr.commandStartStop = C_START;
                    R_STATES |= (1<<1);                     //состояние компрессора = 1
                    CompresStr.timerMain = 0;               //сбрасываем таймер
                    CompresStr.stepNum = NEXT_OFF;          //выходим из первоначального состояния
                }
                break;
                
            //повторное выключение компрессора
            case NEXT_OFF:
                R_STATES &=~ (1<<8);                        //состояния задержки на включение
                //если прошло 3 минуты и t внутр. <= t заданной - Δt
                //или если t внутр. <= t(h1) срочного выключения
                if (((CompresStr.timerMain>=DEF_3MIN) && (getValueSensor(BUF_T3) <= (W_T_INT - W_DT_INT))) 
                || (getValueSensor(BUF_T3) <= W_TQOFF)){
                    CompresStr.commandStartStop = C_STOP;
                    R_STATES &=~(1<<1);                     //состояние компрессора = 0
                    CompresStr.timerOff = 0;                //сбрасываем таймер OFF
                    CompresStr.stepNum = NEXT_ON;           //переход к состоянию NEXT_ON
                }
                break;

            //повторное включение компрессора
            case NEXT_ON:
                //проверка состояния задержки на включение
                if ((CompresStr.timerMain <= DEF_7MIN) && (getValueSensor(BUF_T3) >= W_T_INT)){
                    R_STATES |= (1<<8);				              //состояния задержки на включение
                } else {
                    R_STATES &=~ (1<<8);
                }
                //если прошло 7 мин. от последн. вкл. и t внутр. >= t заданной 
                //или если t внутр. >= t(a1) срочного включения
                //и если прошла обязательная задержка
                if ((((CompresStr.timerMain>=DEF_7MIN) && (getValueSensor(BUF_T3) >= W_T_INT))
                || (getValueSensor(BUF_T3) >= W_TQON))
                && CompresStr.timerOff >= DEF_1MIN){
                    CompresStr.commandStartStop = C_START;
                    R_STATES |= (1<<1);                     //состояние компрессора = 1
                    CompresStr.timerMain = 0;               //сбрасываем таймер
                    CompresStr.stepNum = NEXT_OFF;          //переход к состоянию NEXT_OFF
                }
                break;
            default:
                break;
        }
    } else {
        R_STATES &=~ (1<<8);                                //состояние задержки на включение OFF
        CompresStr.commandStartStop = C_STOP;
        R_STATES &=~(1<<1);                                 //состояние компрессора = 0
        CompresStr.timerMain = 0;                           //сбрасываем таймер
        CompresStr.timerOff = 0;                            //сбрасываем таймер OFF
        CompresStr.stepNum = FIRST_ON;                      //переход к состоянию NEXT_ON
    }
}


//*****************************************************************************
//Управление ТЭНа
//*****************************************************************************
void controlTen(void) 
{
  //если t втутр. <= t задан. вкл. ТЕНа
  //и выключен компрессор
  //и нет ошибки датчика температуры t3
  //и нет аварии по питанию +48V
  //и нет аварии вентилятора испарителя
  //и если вентилятор исп. раскрутился
  if ((getValueSensor(BUF_T3) <= W_TON_HEAT)
  && !STATE_COMPRESSOR
  && !getErrorSensor(BUF_T3)
  && !ALARM_48V
  && !ALARM_FAN_I
  /*&& (STATE_FAN_EVAP && (R_RPM_FAN_I > 1000))*/){
      GPIO_SetBits(GP_RELAY_HEAT_BOX);    //включаем тэн подоргрева шкафа
      R_STATES |= (1<<4);							    //состояние тэна подогрева шкафа = 1
  }
  //если t внутр. > t задан. вкл. ТЕНа + 3
  //или ошибка датчика t3
  //или включён компрессор
  //или есть авария вентилятора испарителя
  //или есть авария по питанию +48V
  //или если вентилятор исп. останавливается
  else if ((getValueSensor(BUF_T3) >= (W_TON_HEAT + 3))
  || getErrorSensor(BUF_T3)
  || STATE_COMPRESSOR
  || ALARM_FAN_I
  || ALARM_48V
  || !STATE_FAN_EVAP 
  /*|| (R_RPM_FAN_I < 600)*/){
      GPIO_ResetBits(GPIOB, PIN_RELAY_HEAT_BOX);		//выключаем тэн подргрева шкафа
      R_STATES &=~ (1<<4);                          //состояние тэна подогрева шкафа = 0
  }
}

//*****************************************************************************
//Управление вентилятора конденсатора
//*****************************************************************************
void controlFanC(void)
{
    //Если включился компрессор и не прошло время задержки "перехода в рабочий режим"
	if (((STATE_COMPRESSOR) && (FanCondenserStr.timerStart < DEF_DELAY_START_FAN_COND))
    && getValueSensor(BUF_T7) < W_TZPPK) {
        //если t7 больше или равна +20  и меньше +25
		//и нет ошибок датчика
		if (((getValueSensor(BUF_T7) >= 20)&&(getValueSensor(BUF_T7) < 25))
		&& !getErrorSensor(BUF_T7)) {
			DUTY_SPEED_FAN_C = MIN_DUTY_SPEED_FAN;                  //устанавливаем скорость вращения MIN
		}
		//если t7 больше или равна +25  и меньше +35
		//и нет ошибок датчика
		else if (((getValueSensor(BUF_T7) >= 25)&&(getValueSensor(BUF_T7) < 35))
		&& !getErrorSensor(BUF_T7)) {
			DUTY_SPEED_FAN_C = (50 * MAX_DUTY_SPEED_FAN) / 100;     //устанавливаем скорость вращения на 50%
		}
		//если t7 не минусовая и больше или равна +35
		//и нет ошибок датчика
		else if ((getValueSensor(BUF_T7) >= 35)
		&& !getErrorSensor(BUF_T7)) {
			DUTY_SPEED_FAN_C = (100 * MAX_DUTY_SPEED_FAN) / 100;    //устанавливаем скорость вращения на 100%
		}
	} else if (CompresStr.timerOff < 1800 || STATE_COMPRESSOR){
		//если t7 >= t зоны "++" конденсатора
		//и нет ошибок датчика температуры
		if((getValueSensor(BUF_T7) >= W_TZPPK)
		&& !getErrorSensor(BUF_T7)){
        funcRunDelayed (NT_CONTROL_FANC, W_TIMEZPPK);
		}
		
		//если t7 >= t зоны "+" конденсатора
		//и нет ошибок датчика температуры
		else if((getValueSensor(BUF_T7) >= W_TZPK)
		&& !getErrorSensor(BUF_T7)){
			if(DUTY_SPEED_FAN_C == 0) DUTY_SPEED_FAN_C = MIN_DUTY_SPEED_FAN;
        funcRunDelayed (NT_CONTROL_FANC, W_TIMEZPK);
		}
		
		//если t7 < t зоны "-" конденсатора
		//и t7 > t зоны "--"
		//и нет ошибок датчика температуры
		else if(((getValueSensor(BUF_T7) < W_TZMK)
		&& (getValueSensor(BUF_T7) > W_TZMMK))
		&& (DUTY_SPEED_FAN_C != 0)
		&& !getErrorSensor(BUF_T7)){
        funcRunDelayed (NT_CONTROL_FANC, W_TIMEZMK);
		}
		
		//если t7 < t зоны "--" конденсатора
		//и нет ошибок датчика температуры
		else if((getValueSensor(BUF_T7) < W_TZMMK)
		&& !getErrorSensor(BUF_T7)){
			DUTY_SPEED_FAN_C = MIN_DUTY_SPEED_FAN;  //устанавливаем мин. скорость
		}
  }
    
	//если компрессор выключен 
  //и t7 меньше зоны "+"
	if (!STATE_COMPRESSOR 
    && (getValueSensor(BUF_T7) < W_TZPK)) {
        FanCondenserStr.timerStart = 0;             //сбрасываем таймер вентилятора конденсатора
        if (CompresStr.timerOff < 1800) {
            DUTY_SPEED_FAN_C = MIN_DUTY_SPEED_FAN;  //устанавливаем мин. скорость
        } else {
            DUTY_SPEED_FAN_C = 0;                   //выкл. вентилятор
            CompresStr.timerOff = 1800;
        }
        //если первое включение компрессора
        if (CompresStr.stepNum == FIRST_ON) {
            DUTY_SPEED_FAN_C = 0;                   //выкл. вентилятор
        }
	}

    //Если ошибка датчика T7
	if (getErrorSensor(BUF_T7)) {
        DUTY_SPEED_FAN_C = MAX_DUTY_SPEED_FAN;  //скорость вращения вент. конденсатора на max.
	}
    
  //если ошибка по питанию - выключаем
  if (ALARM_48V) {
      DUTY_SPEED_FAN_C = 0;
  }
	//если заданная мощность (%) вент. не равна 0
	if (DUTY_SPEED_FAN_C != 0) {
		R_STATES |= (1<<7);                         //состояние вент. конденсатора = 1
	} else {
		R_STATES &=~ (1<<7);                        //состояние вент. конденсатора = 0
	}
	R_FAN_CONDENSER = (uint16_t)(DUTY_SPEED_FAN_C / (MAX_DUTY_SPEED_FAN/100));   //записываем скорость вентелятора конденсатора (%) в карту
}


//*****************************************************************************
//Управление вентилятора испарителя
//*****************************************************************************
void controlfanE(void)
{
    // если компрессор включен
    // и если T6 >= макс. температуры на T6
    // и нет ошибки датчика T6
    if(STATE_COMPRESSOR
    && (getValueSensor(BUF_T6) >= TEMP_T6_MAX)
    && !getErrorSensor(BUF_T6)) {
        DUTY_SPEED_FAN_I = (uint16_t)(((uint32_t) 40 * MAX_DUTY_SPEED_FAN)/100);    //скорость вент. на 40%
    } 
    // иначе если компрессор включен
    // и если T6 < мин. температуры на T6
    else if (STATE_COMPRESSOR && (getValueSensor(BUF_T6) < TEMP_T6_MIN)){
        funcRunDelayed (NT_CONTROL_FANI, 1000); // увеличиваем плавно обороты
    // иначе если компрессор выключен
    } else if (!STATE_COMPRESSOR){
        DUTY_SPEED_FAN_I = (uint16_t)(((uint32_t) 40 * MAX_DUTY_SPEED_FAN)/100);    //скорость вент. на 40%
    }
    if (ALARM_48V) {
        DUTY_SPEED_FAN_I = 0;
        R_FAN_EVAP = 0;
    }
    //если скорость вент. испарит. не равна 0
    if(DUTY_SPEED_FAN_I != 0) {
        R_STATES |= (1<<6);                         //состояние вент. испарителя = 1
    } else {
        R_STATES &=~ (1<<6);                        //состояние вент. испарителя = 0
    }
    //записываем мощность вентелятора испарителя (%) в карту регистров
    R_FAN_EVAP = (DUTY_SPEED_FAN_I * 100) / MAX_DUTY_SPEED_FAN;	
}


//*****************************************************************************
//Ручное управление в режиме ТЕСТ
//*****************************************************************************
void controlHand(void){
//*********************************				
//ручное управление компрессором
	if((W_COMPRES >= 1 || BIT_COMPRESS) && !ALARM_HI_PRESS){
		CompresStr.commandStartStop = C_START;
		R_STATES |= (1<<1);                         //состояние компрессора = 1
	}
	else if((W_COMPRES == 0 && !BIT_COMPRESS) || ALARM_HI_PRESS){
    CompresStr.commandStartStop = C_STOP;
		R_STATES &=~(1<<1);                         //состояние компрессора = 0
	}
//*********************************
//ручное управление вент. конденсатора
	if (W_FAN_CONDENSER == 0){
		DUTY_SPEED_FAN_C = 0;
    R_STATES &=~ (1<<7);                        //состояние вент. конденсатора. = 0
		R_FAN_CONDENSER = 0;				                //записываем мощность вентелятора конденсатора (%) в карту
	} else {
		DUTY_SPEED_FAN_C = ((W_FAN_CONDENSER * MAX_DUTY_SPEED_FAN) / 100);
    R_STATES |= (1<<7);                         //состояние вент. конденсатора. = 1
		R_FAN_CONDENSER = W_FAN_CONDENSER;          //записываем мощность вентелятора конденсатора (%) в карту
	}	

//*********************************
//ручное управление вент. испарителя
  if (W_FAN_EVAP == 0){
    DUTY_SPEED_FAN_I = 0;
    R_STATES &=~ (1<<6);                        //состояние вент. испарителя = 0  
    R_FAN_EVAP = 0;	                            //записываем мощность вентелятора испарителя (%) в карту регистров
  } else {
    DUTY_SPEED_FAN_I = ((W_FAN_EVAP * MAX_DUTY_SPEED_FAN) / 100);
    R_STATES |= (1<<6);                         //состояние вент. испарителя = 1
    R_FAN_EVAP = W_FAN_EVAP;	                  //записываем мощность вентелятора испарителя (%) в карту регистров
	}
	
//*********************************					
//ручное управление подогрева шкафа
	if (BIT_HEAT_BOX && !STATE_COMPRESSOR && (DUTY_SPEED_FAN_I > MIN_DUTY_SPEED_FAN)){
    GPIO_SetBits(GP_RELAY_HEAT_BOX);            //включаем подогрев шкафа
		R_STATES |= (1<<4);                         //состояние подогрева шкафа = 1
	} else {
		GPIO_ResetBits(GP_RELAY_HEAT_BOX);          //выключаем подогрев шкафа
		R_STATES &=~(1<<4);                         //состояние подогрева шкафа = 0
	}
//*********************************					
//ручное управление вент. водорода
	if (BIT_FAN_H2){
    GPIO_SetBits(GP_RELAY_FAN_H2);              //включаем вент. водорода
		R_STATES |= (1<<9);                         //состояние вент. водорода = 1
	} else {
		GPIO_ResetBits(GP_RELAY_FAN_H2);            //выключаем вент. водорода
		R_STATES &=~(1<<9);                         //состояние вент. водорода = 0
	}			
//***************************************************************************
//Обработка аварийных ситуаций в режиме "ТЕСТ"
//***************************************************************************
//Если запросы по внешнему ModBus закончились (прошло 5 тиков)
//и установлен бит режима "ТЕСТ"
	if((modBusExt.countReqTest > 5) && BIT_TEST){
		res_table.regsF1_5[0] &=~ (1<<1);           //выход из режима "ТЕСТ"
	}
}

