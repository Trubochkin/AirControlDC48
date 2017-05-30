// Header: Conditioner control
// File Name: main.c
// Author:  Trubochkin Alexey
// Date:  28.05.2017


/* Includes ------------------------------------------------------------------*/
#include "main_ini.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Global variables ---------------------------------------------------------*/
const char CompanyID[4] = "TECO";
const char ProdCode[7] = "Aircool";
const char Version[11] = "1.0.0-F4P";

uint8_t flagHandControl;                //флаг ручного управления
uint8_t flagAutoControl;                //флаг автоматического управления

UartDataTypeDef modBusInt, modBusExt;				
typeDef_table res_table;
TimersTypeDef TimersStr;                //параметры программных таймеров                       
TempValTypeDef TempStr;                 //параметры для чтения температур
ADCparametersTypeDef ADCStr;            //параметры ADC
CompressorTypeDef CompresStr;           //параметры компрессора
FANCondenserTypeDef FanCondenserStr;    //параметры вентилятора конденсатора
FANEvaporatorTypeDef FanEvaporatorStr;  //параметры вентилятора испарителя


/* Private function prototypes -----------------------------------------------*/
void write_value_default(uint8_t i);

/* Private functions ---------------------------------------------------------*/
//Сброс параметрируемых значений в default
//i - тип сброса (0-после первой прошивки, 1-повторный)
void write_value_default(uint8_t i)
{
  W_T_INT = 30;                   // Внутренняя температура (tвн)
  W_DT_INT = 3;                   // Δt вкл/выкл компрессора
  W_TQON = 32;                    // Температура(ta1) для срочного включения компрессора игнорируя задержку
  W_TAH = 40;                     // Аварийно-высокая температура в шкафу (ta2)
  W_TQOFF = 18;                   // Температура(th1) срочного выключения компрессора игнорируя задержку
  W_TAL = 7;                      // Аварийно-низкая температура в шкафу (th2)
  W_ADDRESS = 1;                  // Адрес устройства	для ВНЕШНЕГО ModBus
  W_SPEED = 3;                    // Скорость передачи для ВНЕШНЕГО ModBus
  if(i != 1){
      W_T7_ALARM = 60;            // Аварийная температура хладагента
      W_DT7_ALARM = 5;            // Δt аварийной температуры хладагента
  }
  W_TZPK = 37;                    // Температура для зоны «+» конденсатора
  W_TZPPK = 39;                   // Температура для зоны «++» конденсатора
  W_TZMK = 36;                    // Температура для зоны «-» конденсатора
  W_TZMMK = 24;                   // Температура для зоны «--» конденсатора
  W_TIMEZPK = 200;                // Интервал времени для зоны «+» конденсатора
  W_TIMEZPPK = 30;                // Интервал времени для зоны «++» конденсатора
  W_TIMEZMK = 280;                // Интервал времени для зоны «-» конденсатора
  W_FREEZ_EVAP = -2;              // Температура обмерзания испарителя
  W_DT_FREEZ = 10;                // Гистерезис обмерзания испарителя
  W_ALARM_POWER_H = 58;           // Аварийно-высокое напряжение питания
  W_ALARM_POWER_L = 40;           // Аварийно-низкое напряжение питания
  W_ALARM_T_COMPRES = 90;         // Аварийная темпреатура компрессора
  W_MIN_T_COMPRES = 2;            // Минимально-рабочая темпреатура компрессора
  W_FAN_CONDENSER = 0;
  W_FAN_EVAP = 0;
  W_COMPRES = 0;
  W_MAX_RPM_COMPRESS = 2500;
  W_TON_HEAT = 10;
    
	FLASH_Unlock();
	Flash_Erase(ADDRESS_PAGE_63); 	//стираем 63 страницу в памяти
	Flash_Write((uint8_t*)&res_table.regsF3_6[20], ADDRESS_PAGE_63, 60);	//Сохраняем на FLASH
	FLASH_Lock();
}


int main(void)
{
  //Если параметрируемые значения отсутствуют на FLASH (при первой прошивке)
  if(Flash_Read(ADDRESS_PAGE_63) == 0xFFFFFFFF){
    write_value_default(0);         //сбрасываем значения в default
  }
    
  read_flash_value();               //чтение параметрируемых значений из FLASH
  SysTick_Config(48000);            //системный таймер на отсчёт миллисекунд
  initializeTimerDelay();           //для мкс задержек (TIM6)
//	I2C_Initialization();
  initializeADC();                  //АЦП для напряжения и тока
  initializeCompress();             //компрессор (TIM1)
  initializeTimerTachoCompres();    //таймер для отсчёта периода тахо-сигналов компрессора (TIM15)
  initializeFans();                 //вентиляторы ШИМ (TIM3)
  initializeTachoFans();            //тахометр вентиляторов (TIM16, TIM17)
  initializeRelays();               //реле
  initializeLeds();                 //сетодиоды
  initializeSensorsTemp();          //датчики температуры
    
	
	//Настройка переферии для ModBus
	/*тймауты приёма:
	//2400 	- 150
	//4800 	- 75
	//7200 	- 48
	//9600 	- 38
	//14400	- 25
	//19200 - 19 */
	modBusExt.speed = W_SPEED;        //скорость ВНЕШНИЙ
	modBusExt.adress = W_ADDRESS;     //адрес устройства ВНЕШНИЙ
	/* Задание таймаут приема */
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
	
	modBusInt.delay = 100;            //таймаут приема ВНУТРЕННИЙ
	modBusInt.adress = 1;             //адрес устройства	ВНУТРЕННИЙ

	initializeUSART1ModBusInt();      //инициализация USART-modbus ВНУТРЕНИЙ
	initializeUSART2ModBusExt();      //инициализация USART-modbus ВНЕШНИЙ
	initializeTimerModBusTimeout();   //инициализация таймера для таймаут приема (TIM14)

#ifndef DEBUG
	initializeWatchdog(4, 800);       //инициализация сторожевого таймера на 1 сек. (625)
#endif
  
  R_T1 = -1;
  R_T2 = -1;
  R_T4 = -1;
  R_T5 = -1;
  R_STATES |= (1<<0);               //состояние 220 вкл.
  //сброс значений для теста
  W_FAN_CONDENSER = 0;
  W_FAN_EVAP = 0;
  W_COMPRES = 0;
  
  //установка первоначальных значений температур
	BUF_T3 = 250;
	BUF_T6 = 250;
	BUF_T7 = 250;
	BUF_T8 = 250;  
//  BUF_T1 = 320;
//	BUF_T4 = 120;  
    
  while (1)
  {
/* Для отладки
//	R_ALARM |= (1<<0);      //есть авария - высокая t внутренняя
//	R_ALARM |= (1<<1);      //есть авария - низкая t внутренняя
//	R_ALARM |= (1<<2);      //есть авария - высокое давление
//	R_ALARM |= (1<<3);      //есть авария - неисправность кондиционера
//	R_ALARM |= (1<<4);      //есть авария - неисправность компрессора
//	R_ALARM |= (1<<7);      //есть авария - нет DC48
//	R_ALARM |= (1<<8);      //есть авария - обмерзание испарителя


//	R_STATES |= (1<<0);     //напряжение 220 вольт
//	R_STATES |= (1<<1);     //компрессор
//	R_STATES |= (1<<4);     //подогрев шкафа
//	R_STATES |= (1<<6);     //вентилятор испарителя
//	R_STATES |= (1<<7);     //вентилятор конденсатора
*/

    resetWatchdog ();                                 // сброс сторожевого таймера
    computeValueADC ();                               // вычисление значений напряжения и тока
    
    // выполнение функций с заданной задержкой выполнения (в ms)
    funcRunDelayed (NT_CHECK_ERR, 1000);              // обработка ошибок (аварий)
    funcRunDelayed (NT_TEMP_VAL, 500);                // обновление температур
    funcRunDelayed (NT_RPM_FANI, 400);                // обновление скорости FAN испарителя
    funcRunDelayed (NT_RPM_FANC, 600);                // обновление скорости FAN конденсатора
    funcRunDelayed (NT_RPM_COMPR, 100);               // обновление скорости компрессора
    funcRunDelayed (NT_LED_ALARM, 200);               // обновление состояния светодиода аварии
    funcRunDelayed (NT_LED_STAT, 500);                // обновление светодиода состояния
    funcRunDelayed (NT_RUN_1S, 1000);                 // функция для запуска 1 раз в секунду
    funcRunDelayed (NT_UPDATE_COMPRESS_STATE, 80);    // обновление состояния компрессора
    if(W_SPEED != modBusExt.speed || modBusExt.adress != W_ADDRESS) {
      funcRunDelayed (NT_CHANGE_MODB, 1000);          // обновление параметров modbus
    }
    
    //если режим ТЕСТ выкл. - управление автоматическое
    if (!BIT_TEST){
      if (!flagAutoControl){
          flagHandControl = 0;
          flagAutoControl = 1;
          if (STATE_COMPRESSOR){                      //выход из режима ТЕСТ в рабочий режим
              CompresStr.stepNum = NEXT_OFF;
              CompresStr.timerMain = 0;               //сбрасываем таймер
          } else {
              CompresStr.stepNum = FIRST_ON;
              CompresStr.timerOff = 0;                //сбрасываем таймер OFF
          }
      }
      // если нет аварии об неисправности кондиционера
      if (!ALARM_AIRCOOL_FAULTY){
          controlCompress();                          //логика управления компрессора
          controlFanC();                              //логика управления вент. конденсатора
          controlfanE();                              //логика управления вент. испарителя
          controlTen();                               //логика управления ТЭНа
      } else {
          DUTY_SPEED_COMPRES = MAX_DUTY_COMPRES;      //выключаем компрессор (инверсное управление)
          R_STATES &=~(1<<1);											
          GPIOB->BRR = PIN_RELAY_HEAT_BOX;            //выключаем подогрев шкафа
          R_STATES &=~(1<<4);											
          DUTY_SPEED_FAN_C = 0;                       //выключаем FAN конденсатора
          R_FAN_CONDENSER = 0;
          R_STATES &=~ (1<<7);							
      }
    } 
    //иначе если режим ТЕСТ вкл. - управление вручную
    else {
        if(!flagHandControl){
            flagHandControl = 1;
            flagAutoControl = 0;
        }
//        CompresStr.commandStartStop = C_TEST;
        controlHand();
    }


    //Если параметрируемые значения изменились
    if (	*(__IO uint32_t*)&res_table.regsF3_6[20] != Flash_Read(ADDRESS_PAGE_63)
    ||*(__IO uint32_t*)&res_table.regsF3_6[22] != Flash_Read(ADDRESS_PAGE_63+4)
    ||*(__IO uint32_t*)&res_table.regsF3_6[24] != Flash_Read(ADDRESS_PAGE_63+8)
    ||*(__IO uint32_t*)&res_table.regsF3_6[26] != Flash_Read(ADDRESS_PAGE_63+12)
    ||*(__IO uint32_t*)&res_table.regsF3_6[28] != Flash_Read(ADDRESS_PAGE_63+16)
    ||*(__IO uint32_t*)&res_table.regsF3_6[30] != Flash_Read(ADDRESS_PAGE_63+20)
    ||*(__IO uint32_t*)&res_table.regsF3_6[32] != Flash_Read(ADDRESS_PAGE_63+24)
    ||*(__IO uint32_t*)&res_table.regsF3_6[34] != Flash_Read(ADDRESS_PAGE_63+28)
    ||*(__IO uint32_t*)&res_table.regsF3_6[36] != Flash_Read(ADDRESS_PAGE_63+32)
    ||*(__IO uint32_t*)&res_table.regsF3_6[38] != Flash_Read(ADDRESS_PAGE_63+36)
    ||*(__IO uint32_t*)&res_table.regsF3_6[40] != Flash_Read(ADDRESS_PAGE_63+40)
    ||*(__IO uint32_t*)&res_table.regsF3_6[42] != Flash_Read(ADDRESS_PAGE_63+44)
    ||*(__IO uint32_t*)&res_table.regsF3_6[44] != Flash_Read(ADDRESS_PAGE_63+48)
    ||*(__IO uint32_t*)&res_table.regsF3_6[46] != Flash_Read(ADDRESS_PAGE_63+52)
    ||*(__IO uint32_t*)&res_table.regsF3_6[48] != Flash_Read(ADDRESS_PAGE_63+56)){
        write_flash_value();                //записываем новые значения на FLASH
    }
		
    //Если выставлен бит "Reset"
    if (BIT_RESET&0x01){
        write_value_default(1);             //Сбрасываем значения в default
        R_ALARM &=~(1<<3);                  //сброс аварии неисправности кондиционера
//        FanEvaporatorStr.timerCheck = 0;
        res_table.regsF1_5[0] &=~ (1<<0);   //Сбрасываем бит "Reset"
    }
  }
}

