

#include "f_air_alarm.h"



//*****************************************************************************
//Настройка и перезагрузка независимого сторожевого таймера
//*****************************************************************************
//Tout=((4*2^prer)*rlr)/40 (ms).
void initializeWatchdog(uint8_t prer,uint16_t rlr)
{
	IWDG->KR=0X5555;			//Ключ для доступа к таймеру
	IWDG->PR=prer;  			//Обновление предделителя
	IWDG->RLR=rlr;  			//Загружаем регистр перезагрузки, счет идет вниз от этого значения
	IWDG->KR=0XAAAA;			//перезагрузка
	IWDG->KR=0XCCCC;			//Запуск в работу
}

//Перезагрузка сторожевого таймера
void resetWatchdog (void)
{
	IWDG->KR=0XAAAA;			//перезагрузка
}



//*****************************************************************************
//Обработка ошибок (аварий)
//функция должна выполняться 1 раз в секунду для соблюдения необходимых задержек
//*****************************************************************************
void checkError (void)
{
    //отслеживание -48
    if (((R_POWER_VOLT / 100) >= W_ALARM_POWER_H) || ((R_POWER_VOLT / 100) <= W_ALARM_POWER_L)){
        R_ALARM |= (1<<7);
    } else {
        R_ALARM &=~ (1<<7);
    }
    
    
    //ошибка датчика(ов)
    //*******************************************************************************************************	
    //если есть хоть одна ошибка датчика
    if (/*getErrorSensor(BUF_T1) ||*/
        /*getErrorSensor(BUF_T4) ||*/
        getErrorSensor(BUF_T3) ||
        getErrorSensor(BUF_T7) ||
        getErrorSensor(BUF_T8) ||
        getErrorSensor(BUF_T6)) {
            R_ALARM |= (1<<14);                 //есть ошибка датчика
	} else {
        R_ALARM &=~(1<<14);                     //нет ошибки
    }
    
    
    //высокая t внутренняя
    //*******************************************************************************************************	
    //если t внутр. >= t(a2) аварийно-высокой 
    //и нет ошибок датчика
    if ((getValueSensor(BUF_T3) >= W_TAH)	
        && (!getErrorSensor(BUF_T3))) {
        R_ALARM |= (1<<0);                          //есть авария - высокая t внутренняя
	}
    //иначе если есть авария высокая t внутренняя
    //и если t внутр. <= (t(a2) - DIF_ALARM_HI)
    //или есть ошибка датчика
    else if ((ALARM_HI_INT_TEMP
    && (getValueSensor(BUF_T3) <= (W_TAH - DEF_DIF_ALARM_HI)))
    || getErrorSensor(BUF_T3)){
        R_ALARM &=~(1<<0);                          //нет аварии
    }
	
	
    //низкая t внутренняя
    //*******************************************************************************************************	
    //если t внутр. <= t(h2) аварийно-низкой 
    //и нет ошибок датчика
    //или минусовая температура
    if (((getValueSensor(BUF_T3) <= W_TAL)		
    && !getErrorSensor(BUF_T3))){
        R_ALARM |= (1<<1);                          //есть авария - низкая t внутренняя
    }
    //иначе если есть авария - низкая t внутренняя
    //и если t внутр. >= (t(h2) + DIF_ALARM_LO)
    //или есть ошибки датчика
    else if ((ALARM_LO_INT_TEMP
    && (getValueSensor(BUF_T3) >= (W_TAL + DEF_DIF_ALARM_LO)))
    || getErrorSensor(BUF_T3)){
        R_ALARM &=~(1<<1);                          //нет аварии
    }
	

    //высокое давление
    //*******************************************************************************************************
    //если t на сливе конд. >= заданной авар. t хладагента 
    //и нет ошибок датчика
    if ((getValueSensor(BUF_T7) >= W_T7_ALARM)
    && !getErrorSensor(BUF_T7)){
        R_ALARM |= (1<<2);					        //есть авария - высокое давление
    }
    //иначе если есть авария - высокое давление
    //и если t на сливе конд. <= заданной авар. t хладагента - Δt
    //или есть ошибки датчика
    else if ((ALARM_HI_PRESS
    && (getValueSensor(BUF_T7) <= (W_T7_ALARM - W_DT7_ALARM)))
    || getErrorSensor(BUF_T7)){
        R_ALARM &=~(1<<2);				            //нет аварии
    }


    //неисправность компрессора
    //*******************************************************************************************************	
    //если t компрессора >= DEF_ALARM_TEMP_COMPRES
    //и нет ошибок датчика
    if ((getValueSensor(BUF_T8) >= W_ALARM_T_COMPRES)
    && !getErrorSensor(BUF_T8)){
        R_ALARM |= (1<<4);                      //есть авария - перегрев компрессора
    }
    //иначе если есть авария - перегрев компрессора
    //и если t картера. <= (авар. t компрес.) - (диф. авар. t компрес.)
    //или есть ошибка датчика
    else if ((ALARM_COMPRES_FAULTY
    && (getValueSensor(BUF_T8) <= (W_ALARM_T_COMPRES - DEF_DIF_ALARM_TEMP_COMPRES)))
    || getErrorSensor(BUF_T8)){
        R_ALARM &=~(1<<4);                      //нет аварии
    }


    //обмерзание испарителя
    //*******************************************************************************************************		
    //если t6 на трубе испарителя <= задан. t обмерзания испарителя
    //и нет ошибок датчика
    if ((getValueSensor(BUF_T6) <= W_FREEZ_EVAP)
        && !getErrorSensor(BUF_T6)){
        R_ALARM |= (1<<8);					    //есть авария - обмерзание испарителя
    }
    //иначе если есть авария обмерзание испарителя
    //и если t6 >= (заднан. t обмерзания испарит. + Δt)
    //или есть ошибка датчика
    else if ((ALARM_FREEZ_EVAP
    && getValueSensor(BUF_T6) >= W_FREEZ_EVAP + W_DT_FREEZ)
    || getErrorSensor(BUF_T6)){
        R_ALARM &=~(1<<8);				    //нет аварии
    }


//    //неисправность вентилятора испарителя
//    //*******************************************************************************************************		
//    //если задана команда на включение вентилятора испарителя 
//    //и RPM < MIN_RPM_FANS_ALARM
//    //и нет аварии вентилятора испарителя
//    if (STATE_FAN_EVAP 
//    && R_RPM_FAN_I < MIN_RPM_FANS_ALARM
//    && !ALARM_FAN_I) {
//        FanEvaporatorStr.countToAlarmSet++;       //инкремент 1 раз в секунду
//        FanEvaporatorStr.countToAlarmReset = 0;
//        //если счётчик больше чем заданное время
//        if (FanEvaporatorStr.countToAlarmSet > DEF_DELAY_ALARM_FANS) {
//            R_ALARM |= (1<<9);					    //есть авария - неисправность вентилятора испарителя
//            FanEvaporatorStr.countToAlarmSet = 0;
//        }
//    }
//    //иначе если датчик оборотов >= MIN_RPM_FANS_ALARM
//    //или задана команда на выключение вентилятора испарителя
//    else if ((R_RPM_FAN_I >= MIN_RPM_FANS_ALARM)
//    || !STATE_FAN_EVAP){
//        //если задана команда на выключение вентилятора испарителя
//        if (STATE_FAN_EVAP == 0) {
//            R_ALARM &=~(1<<9);                      //сброс аварии
//            FanEvaporatorStr.countToAlarmSet = 0;
//            FanEvaporatorStr.countToAlarmReset = 0;
//        } else {
//            FanEvaporatorStr.countToAlarmReset++;       //инкремент 1 раз в секунду
//            FanEvaporatorStr.countToAlarmSet = 0;
//            //если счётчик больше чем заданное время
//            if (FanEvaporatorStr.countToAlarmReset > DEF_DELAY_ALARM_FANS) {
//                R_ALARM &=~(1<<9);                      //сброс аварии
//                FanEvaporatorStr.countToAlarmReset = 0;
//            }
//        }
//    }
//    //иначе иесли датчик оборотов < MIN_RPM_FANS_ALARM и есть авария вентилятора испарителя
//    else if ((R_RPM_FAN_I < MIN_RPM_FANS_ALARM) && ALARM_FAN_I) {
//        FanEvaporatorStr.countToAlarmReset = 0;
//        
//    }


//    //неисправность вентилятора конденсатора
//    //*******************************************************************************************************		
//    //если задана команда на включение вентилятора конденсатора
//    //и RPM < MIN_RPM_FANS_ALARM  
//    //и нет аварии вентилятора конденсатора
//    if (STATE_FAN_CONDENSER 
//    && R_RPM_FAN_C < MIN_RPM_FANS_ALARM 
//    && !ALARM_FAN_C){
//        FanCondenserStr.countToAlarmSet++;       //инкремент 1 раз в секунду
//        FanCondenserStr.countToAlarmReset = 0;
//        //если счётчик больше чем заданное время
//        if (FanCondenserStr.countToAlarmSet > DEF_DELAY_ALARM_FANS) {
//            R_ALARM |= (1<<10);					    //есть авария - неисправность вентилятора конденсатора
//            FanCondenserStr.countToAlarmSet = 0;
//        }
//    }
//    //иначе если датчик оборотов >= MIN_RPM_FANS_ALARM
//    //или задана команда на выключение вентилятора конденсатора
//    else if ((R_RPM_FAN_C >= MIN_RPM_FANS_ALARM)
//    || !STATE_FAN_CONDENSER){
//        //если задана команда на выключение вентилятора конденсатора
//        if (STATE_FAN_CONDENSER == 0) {
//            R_ALARM &=~(1<<10);                      //сброс аварии
//            FanCondenserStr.countToAlarmSet = 0;
//            FanCondenserStr.countToAlarmReset = 0;
//        } else {
//            FanCondenserStr.countToAlarmReset++;       //инкремент 1 раз в секунду
//            FanCondenserStr.countToAlarmSet = 0;
//            //если счётчик больше чем заданное время
//            if (FanCondenserStr.countToAlarmReset > DEF_DELAY_ALARM_FANS) {
//                R_ALARM &=~(1<<10);                      //нет аварии
//                FanCondenserStr.countToAlarmReset = 0;
//            }
//        }
//    }
//    //иначе если датчик оборотов < MIN_RPM_FANS_ALARM и есть авария вентилятора конденсатора
//    else if ((R_RPM_FAN_C < MIN_RPM_FANS_ALARM) && ALARM_FAN_C) {
//        FanCondenserStr.countToAlarmReset = 0;
//    }
}
