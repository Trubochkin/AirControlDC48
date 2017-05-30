/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TD_DATA_H
#define __TD_DATA_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
/* Private typedef -----------------------------------------------------------*/

//***************************************************************************
//Дефайны для отладки
//***************************************************************************
//#define DEBUG
#ifdef DEBUG
    #define DEBUG_TEMPERATURE
    #define DEBUG_FANS
		#define DEBUG_COMPRESS
#endif


//***************************************************************************
//Дефайны используемых портов
//***************************************************************************
//Температурные датчики
//#define GP_T1     GPIOB, GPIO_Pin_14        //t1 окружающая
#define GP_T3     GPIOB, GPIO_Pin_15        //t3 на входе испарителя (внутренняя)
#define GP_T6     GPIOA, GPIO_Pin_8         //t6 на всасывающей трубе испарителя
#define GP_T7     GPIOA, GPIO_Pin_9         //t7 на сливе конденсатора
#define GP_T8     GPIOA, GPIO_Pin_10        //t8 компрессора

#define PIN_RELAY_HEAT_BOX      GPIO_Pin_0                  //пин для вкл/выкл реле для тэна обогрева шкафа
#define GP_RELAY_HEAT_BOX       GPIOB, PIN_RELAY_HEAT_BOX   //порт для вкл/выкл реле для тэна обогрева шкафа
#define PIN_RELAY_ALARM         GPIO_Pin_1                  //пин для вкл/выкл реле аварии
#define GP_RELAY_ALARM          GPIOB, PIN_RELAY_ALARM      //порт для вкл/выкл реле аварии
#define PIN_RELAY_FAN_H2        GPIO_Pin_2                  //пин для вкл/выкл реле вентилятора водорода
#define GP_RELAY_FAN_H2         GPIOB, PIN_RELAY_FAN_H2     //порт для вкл/выкл реле вентилятора водорода

#define GP_TACHO1               GPIOB, GPIO_Pin_8           //пин для тахометра вентилятора конденсатора
#define GP_TACHO2               GPIOB, GPIO_Pin_9           //пин для тахометра вентилятора испарителя

#define GP_LED_STAT             GPIOC, GPIO_Pin_13          //пин для светодиода состояния
#define GP_LED_ERR              GPIOC, GPIO_Pin_14          //пин для светодиода ошибок


//***************************************************************************
//Дефайны для работы с вентиляторами
//***************************************************************************
#define MAX_DUTY_CYCLE      TIM3->ARR       //Максимальное значение DUTY CYCLE для ШИМ вентиляторов
#define MIN_DUTY_SPEED_FAN  320             //Значение min скорости для вентеляторов (20 %)
#define MAX_DUTY_SPEED_FAN  1600            //Значение max скорости для вентеляторов (100 %)
#define DUTY_SPEED_FAN_C    TIM3->CCR1      //Значение скорости вентелятора конденсатора
#define DUTY_SPEED_FAN_I    TIM3->CCR2      //Значение скорости вентелятора испарителя
#define MIN_RPM_FANS_ALARM          400     //аварийно-минимальные обороты вентиляторов (RPM)
#define DEF_DELAY_START_FAN_COND    30      //время задержки стартового режима для вентилятора конденсатора	(в сек.)
#define DEF_DELAY_ALARM_FANS        10      //время задержки до установки/сброса события аварии вентиляторов(в сек.)
//#define DEF_DELAY_ALARM_COND        1800    //время задержки на отработку аварии неисправности кондиционера (в сек.)
//#define DEF_DELAY_CHECK_FAN_EVAP    120     //время проверки работы вентилятора испарителя (сек.)
#define TEMP_T6_MIN         10
#define TEMP_T6_MAX         16

//***************************************************************************
//Дефайны для работы с компрессором
//***************************************************************************
#define MAX_DUTY_COMPRES    (TIM1->ARR+1)   //Максимальное значение DUTY CYCLE для ШИМ компрессора
#define DUTY_SPEED_COMPRES  TIM1->CCR4      //Значение DUTY CYCLE скорости компрессора (инверсное)
#define DEF_DIF_ALARM_TEMP_COMPRES  5       //гистерезис отключения аварии компрессора	( tC )
#define MIN_P_COMPRES               1       //минимальная мощность компрессора ( % )
#define	DEF_1MIN                    (1*60)  //время задержки (сек.)
#define	DEF_3MIN                    (3*60)  //время задержки (сек.)
#define	DEF_7MIN                    (7*60)  //время задержки (сек.)
#define FIRST_ON                    0
#define NEXT_OFF                    1
#define NEXT_ON                     2
#define C_STOP                      0
#define C_START                     1
#define C_TEST                      2

#define DEF_DIF_ALARM_HI            2       //гистерезис отключения аварии высокой температуры	(в tC)
#define DEF_DIF_ALARM_LO            2       //гистерезис отключения аварии низкой температуры	(в tC)

#define BUF_T1                      TempStr.bufTemp[0]
#define BUF_T3                      TempStr.bufTemp[1]
#define BUF_T4                      TempStr.bufTemp[2]
#define BUF_T6                      TempStr.bufTemp[3]
#define BUF_T7                      TempStr.bufTemp[4]
#define BUF_T8                      TempStr.bufTemp[5]

#define SENSOR_COUNT_ERR            10      //значение счётчика получения аварии датчика

//***************************************************************************
//Дефайны для работы с данными и FLASH
//***************************************************************************
#define ADDRESS_PAGE_63 FLASH_BASE+63*1024	//63 - номер страницы для сохранения настроек


//***************************************************************************
//Дефайны для работы ModBus
//***************************************************************************
/*для функций 3 и 6
0x1xxxxxxx - минусовая температура
0x0xxxxxxx - плюсовая температура
0x10xxxxxxxx - короткое замыкание датчика на землю
0x01xxxxxxxx - обрыв датчика */
#define R_STATES                res_table.regsF3_6[0]   /*!< Регистр состояний */
#define STATE_220V              (R_STATES & 0x01)       /*!< бит состояния напряжения 220 вольт (1 - вкл, 0 - выкл) */
#define STATE_COMPRESSOR        (R_STATES & 0x02)        /*!< бит состояния компрессора (1 - вкл, 0 - выкл) */
#define STATE_HEAT_CARTER       (R_STATES & 0x04)       /*!< бит состояния подогрева картера (1 - вкл, 0 - выкл) */
#define STATE_HEAT_DRAIN        (R_STATES & 0x08)       /*!< бит состояния подогрева дренажа (1 - вкл, 0 - выкл) */
#define STATE_HEAT_BOX          (R_STATES & 0x10)       /*!< бит состояния подогрева шкафа (1 - вкл, 0 - выкл) */
#define STATE_FAN_FCOOL         (R_STATES & 0x20)       /*!< бит состояния вентилятора фрикулинга (1 - вкл, 0 - выкл) */
#define STATE_FAN_EVAP          (R_STATES & 0x40)       /*!< бит состояния вентилятора испарителя (1 - вкл, 0 - выкл) */
#define STATE_FAN_CONDENSER     (R_STATES & 0x80)       /*!< бит состояния вентилятора конденсатора (1 - вкл, 0 - выкл) */
#define STATE_DELAY_ON_C        (R_STATES & 0x100)      /*!< бит состояния задержки на вкл. компрессоа (1 - вкл, 0 - выкл) */
#define STATE_FAN_H2            (R_STATES & 0x200)      /*!< бит состояния вентилятора H2 (1 - вкл, 0 - выкл) */


#define R_ALARM                 res_table.regsF3_6[1]   /*!< Регистр аварий */
#define ALARM_HI_INT_TEMP       (R_ALARM & 0x01)        /*!< бит аварии по высокой внутренней температуре (1 - alarm, 0 - ok) */
#define ALARM_LO_INT_TEMP       (R_ALARM & 0x02)        /*!< бит аварии по низкой внутренней температуре (1 - alarm, 0 - ok) */
#define ALARM_HI_PRESS          (R_ALARM & 0x04)        /*!< бит аварии по высокому давлению (1 - alarm, 0 - ok) */
#define ALARM_AIRCOOL_FAULTY    (R_ALARM & 0x08)        /*!< бит аварии об неисправности кондиционера (1 - alarm, 0 - ok) */
#define ALARM_COMPRES_FAULTY    (R_ALARM & 0x10)        /*!< бит аварии об перегрев компрессора (t8>90) (1 - alarm, 0 - ok) */
#define ALARM_FCOOL_FAULTY      (R_ALARM & 0x20)        /*!< бит аварии об неисправности фрикулинга (1 - alarm, 0 - ok) */
#define ALARM_220V              (R_ALARM & 0x40)        /*!< бит аварии об отсутствии напряжения 220 вольт (1 - alarm, 0 - ok) */
#define ALARM_48V               (R_ALARM & 0x80)        /*!< бит аварии об аварийном напряжения 48 вольт (1 - alarm, 0 - ok) */
#define ALARM_FREEZ_EVAP        (R_ALARM & 0x100)       /*!< бит аварии об обмерзании испарителя (1 - alarm, 0 - ok) */
#define ALARM_FAN_I             (R_ALARM & 0x200)       /*!< бит аварии об неисправности вент. испарителя (1 - alarm, 0 - ok) */
#define ALARM_FAN_C             (R_ALARM & 0x400)       /*!< бит аварии об неисправности вент. конденсатора (1 - alarm, 0 - ok) */

#define REGF3_6             res_table.regsF3_6
#define R_T3                REGF3_6[2]      //Внутреняя температура в шкафу
#define R_T1                REGF3_6[3]      //Температура окруж. среды
#define R_T7                REGF3_6[4]      //Температура на сливе конденсатора
#define R_T8                REGF3_6[5]      //Температура компрессора
#define R_T4                REGF3_6[6]      //Температура на выходе испарителя
#define R_T6                REGF3_6[7]      //Температура на всасывающей трубе компрессора
#define R_T5                REGF3_6[8]      //Температура наружного воздуха на выходе фрикулинга
#define R_T2                REGF3_6[9]      //Температура на входе фрикулинга (внутренний поток)
#define R_FAN_CONDENSER     REGF3_6[10]     //Заданная мощность вентилятора конденсатора  %
#define R_FAN_EVAP          REGF3_6[11]     //Заданная мощность вентилятора испарителя  %
#define R_COMPRESS          REGF3_6[12]     //Скорость вращения компрессора  %
#define R_POWER_VOLT        REGF3_6[13]     //Напряжение питания
#define R_CURRENT_AMP       REGF3_6[14]     //Ток компрессора
#define R_POWER             REGF3_6[15]     //Моность компрессора
#define R_RPM_FAN_C         REGF3_6[16]     //Скорость вращения вентилятора конденсатора RPM
#define R_RPM_FAN_I         REGF3_6[17]     //Скорость вращения вентилятора испарителя RPM
#define R_RPM_COMPRES       REGF3_6[18]     //Скорость вращения компрессора RPM
#define R_SET_SPEED2        REGF3_6[19]
// задание параметра:
#define W_T_INT             REGF3_6[20]      // внутренней температуры (tвн)
#define W_DT_INT            REGF3_6[21]      // гистерезиса вкл/выкл компрессора
#define W_TQON              REGF3_6[22]      // температуры срочного включения компрессора игнорируя все задержки
#define W_TAH               REGF3_6[23]      // аварийно-высокой температуры в шкафу (ta2)
#define W_TQOFF             REGF3_6[24]      // температуры срочного выключения компрессора игнорируя все задержки
#define W_TAL               REGF3_6[25]      // аварийно-низкой температуры в шкафу(th2)
#define W_ADDRESS           REGF3_6[26]      // сетевого адреса для внешнего ModBus
#define W_SPEED             REGF3_6[27]      // скорости обмена для внешнего ModBus
#define W_T7_ALARM          REGF3_6[28]      // аварийной температуры хладагента датчика T7
#define W_DT7_ALARM         REGF3_6[29]      // гистерезиса аварийной температуры хладагента
#define W_TZPK              REGF3_6[30]      // температуры для зоны «+» конденсатора
#define W_TZPPK             REGF3_6[31]      // температуры для зоны «++» конденсатора
#define W_TZMK              REGF3_6[32]      // температуры для зоны «-» конденсатора
#define W_TZMMK             REGF3_6[33]      // температуры для зоны «--» конденсатора
#define W_TIMEZPK           REGF3_6[34]      // интервала времени для зоны «+» конденсатора
#define W_TIMEZPPK          REGF3_6[35]      // интервала времени для зоны «++» конденсатора
#define W_TIMEZMK           REGF3_6[36]      // интервала времени для зоны «-» конденсатора
#define W_TONF              REGF3_6[37]      // температуры для включения фрикулинга
#define W_DTF               REGF3_6[38]      // гистерезиса фрикулинга
#define W_FREEZ_EVAP        REGF3_6[39]      // температуры обмерзания испарителя (T6)
#define W_DT_FREEZ          REGF3_6[40]      // гистерезиса обмерзания испарителя
#define W_ALARM_POWER_H     REGF3_6[41]      // аварийно-высокого напряжения питания
#define W_ALARM_POWER_L     REGF3_6[42]      // аварийно-низкого напряжения питания
#define W_ALARM_T_COMPRES   REGF3_6[43]      // аварийной температуры компрессора
#define W_MIN_T_COMPRES     REGF3_6[44]      // минимальной температуры компрессора
#define W_FAN_CONDENSER     REGF3_6[45]      // скорости вращения вент. конденс. % (только для режима "ТЕСТ")
#define W_FAN_EVAP          REGF3_6[46]      // скорости вращения вент. ипарит. % (только для режима "ТЕСТ")
#define W_COMPRES           REGF3_6[47]      // скорости вращения компрессора % (только для режима "ТЕСТ")
#define W_MIN_RPM_COMPRESS  REGF3_6[48]      // минимальная скорость вращения компрессора
#define W_TON_HEAT          REGF3_6[49]      // температура вкл. ТЕНа обогрева

//для функции 5 и 1
#define BIT_RESET           (res_table.regsF1_5[0]&0x01)    //бит сброса настроек по умолчанию (заводские настройки)
#define BIT_TEST            (res_table.regsF1_5[0]&0x02)    //бит активации режима «ТЕСТ»
#define BIT_TYPE_FAN        (res_table.regsF1_5[0]&0x04)    //бит выбора типа вентиляторов для испарителя и фрикулинга
#define BIT_COMPRESS        (res_table.regsF1_5[1]&0x01)    //бит вкл/выкл компрессора в ручном режиме (только для режима "ТЕСТ")
#define BIT_HEAT_BOX        (res_table.regsF1_5[1]&0x02)    //бит вкл/выкл тэна подогрева шкафа (только для режима "ТЕСТ")
#define BIT_HEAT_DRAIN      (res_table.regsF1_5[1]&0x04)    //бит вкл/выкл тэна дренажа (только для режима "ТЕСТ")
#define BIT_HEAT_CARTER     (res_table.regsF1_5[1]&0x08)    //бит вкл/выкл тэна подогрева картера компрессора (только для режима "ТЕСТ")
#define BIT_FAN_EVAP        (res_table.regsF1_5[1]&0x10)    //бит вкл/выкл вент. испарителя (только для режима "ТЕСТ")
#define BIT_FAN_FCOOL       (res_table.regsF1_5[1]&0x20)    //бит вкл/выкл вент. фрикул. (только для режима "ТЕСТ")
#define BIT_FAN_H2          (res_table.regsF1_5[1]&0x40)    //бит вкл/выкл вент. водорода (только для режима "ТЕСТ")

#define OBJ_SZ_F3_6         52              //количество регистров для функций 3 и 6
#define OBJ_SZ_F1_5         2               //количество байт для функций 1 и 5

//buffer uart
#define BUF_SZ 256                          //размер буфера
#define MODBUS_WRD_SZ       (BUF_SZ-5)/2    //максимальное количество регистров в ответе

//Массив со значениями скоростей
extern uint32_t numSpeed[9];

//Структура массивов то откуда мы читаем и куда пишем
typedef struct{
	int16_t regsF3_6[OBJ_SZ_F3_6];          //регистры чтения и записи для функций 3 и 6
	uint8_t regsF1_5[OBJ_SZ_F1_5];          //регистры чтения и записи для функции 1 и 5
}typeDef_table;
extern typeDef_table res_table;

//Device Identification
const extern char CompanyID[4];
const extern char ProdCode[7];
const extern char Version[11];

//modBus structure
typedef struct {
    uint8_t buffer[BUF_SZ];         //буфер		
    uint16_t rxtimer;               //этим мы считаем таймоут
    uint8_t rxcnt;                  //количество принятых символов
    uint8_t txcnt;                  //количество переданных символов
    uint8_t txlen;                  //длина посылки на отправку
    uint8_t speed;                  //скорость передачи данных
    uint16_t adress;                //адрес ModBus
    uint16_t delay;                 //задержка
    uint8_t countReqTest;           //счётчик запросов для выхода из режима тест
}UartDataTypeDef;
extern UartDataTypeDef modBusInt, modBusExt;	//структуры для внешнего и внутреннего modBus


//***************************************************************************
//Структуры данных для управления кондиционером
//***************************************************************************
extern uint8_t flagHandControl;     //флаг ручного управления
extern uint8_t flagAutoControl;     //флаг автоматического управления

//количество и номера программных таймеров
#define N_TIMERS                    12
#define NT_TEMP_VAL                 0
#define NT_RPM_FANI                 1
#define NT_RPM_FANC                 2
#define NT_RPM_COMPR                3
#define NT_LED_ALARM                4
#define NT_LED_STAT                 5
#define NT_CHANGE_MODB              6
#define NT_CONTROL_FANC             7
#define NT_CONTROL_FANI             8
#define NT_RUN_1S                   9
#define NT_UPDATE_COMPRESS_STATE    10
#define NT_CHECK_ERR                11

//структура параметров для программных таймеров
typedef struct {
    uint16_t counter[N_TIMERS];      //счётчики таймеров
    uint8_t flagCountEnable[N_TIMERS];   //параметры для включения/выключения счёта
} TimersTypeDef;
extern TimersTypeDef TimersStr;	    //параметры программных таймеров


//структура параметров для чтения температур
typedef struct {
    uint16_t bufTemp[8];            //буффер значений температур в дополнительном коде
    uint8_t count85C[9];            //счётчики для определения температуры 85 градусов
    uint8_t countErr[9];            //счётчики ошибок датчиков
} TempValTypeDef;
extern TempValTypeDef TempStr;	    //параметры для чтения температур


//структура параметров АЦП (напряжение / ток)
typedef struct {
    uint16_t buffData[41];          //Буфер значений АЦП для усреднения
    uint16_t voltage;               //Значение напряжения
    uint16_t current;               //Значение тока
} ADCparametersTypeDef;
extern ADCparametersTypeDef ADCStr;	//параметры ADC


//структура параметров для компрессора
typedef struct {
    uint16_t powerPercent;          //заданная мощность	(%)
    uint32_t speedRPM;              //скорость вращения об/мин
    uint16_t timePeriodTurn;        //время одного оборота (в десятках мкс.)
    uint8_t flagCountTimDone;       //флаг завершения счёта таймера для одного оборота
    uint16_t timerMain;             //таймер компрессора (сек.)
    uint32_t timerOff;              //таймер выключенного компрессора (сек.)
    uint16_t timerOn;               //таймер включенного компрессора (сек.)
    uint8_t stepNum;                //номер шага выполнения компрессора
    uint8_t countTacho;             //счётчик тахо импульсов
    uint8_t countDelayOff;          //счётчик для задержки отключения компрессора по аварии
    uint8_t flagConfirm;            //флаг разрешения включения компрессора
    uint8_t countCheckStop; 	    	//счётчик для определения остановки компрессора
    uint8_t buffCheckStop;          //буффер импульсов для определения остановки компрессора
    uint8_t commandStartStop;       //команда старт/стоп (1 - старт, 0 - стоп)
    uint8_t countFault;							//счётчик неудачных стартов компрессора
    uint16_t countTimeStart;        //таймер для старта компрессора
    uint16_t countTimeWork;         //таймер для раскрутки компрессора до рабочих оборотов
    uint16_t countTimeRestart;      //таймер для перезапуска компрессора в случае остановки компрессора
    uint8_t flagFault;              //флаг компрессор упал
    uint8_t flagSetStart;           //флаг успешного старта
} CompressorTypeDef;
extern CompressorTypeDef CompresStr;	//параметры компрессора

//структура параметров для вентилятора конденсатора
typedef struct {
    uint16_t powerPercent;          //заданная мощность	(%)
    uint16_t speedRPM;              //скорость вращения об/мин
    uint16_t buffTacho[2];          //буфер значений периодов для тахо
    uint16_t period;                //период тахо
    uint16_t timerStart;            //таймер стартового режима (сек.)
    uint8_t countToAlarmSet;        //счётчик до установки события аварии вентилятора конденсатора
    uint8_t countToAlarmReset;      //счётчик до сброса события аварии вентилятора конденсатора
    uint8_t countCheckStop;         //счётчик для определения остановки вентилятора
    uint16_t buffCheckStop;         //буффер для определения остановки вентилятора
} FANCondenserTypeDef;
extern FANCondenserTypeDef FanCondenserStr;	//параметры вентилятора конденсатора

//структура параметров для вентилятора испарителя
typedef struct {
    uint16_t powerPercent;          //заданная мощность	(%)
    uint16_t speedRPM;              //скорость вращения об/мин
    uint16_t buffTacho[2];          //буфер значений периодов для тахо
    uint16_t period;                //период тахо
    uint8_t timerStart;             //таймер стартового режима (сек.)
    uint8_t countToAlarmSet;        //счётчик до установки события аварии вентилятора испарителя
    uint8_t countToAlarmReset;      //счётчик до сброса события аварии вентилятора испарителя
    uint8_t countCheckStop;         //счётчик для определения остановки вентилятора
    uint16_t buffCheckStop;         //буффер для определения остановки вентилятора
//    uint8_t timerCheck; 	        //таймер проверки работы вентилятора испарителя (сек.)
//    uint8_t statChecking;           //состояние проверки вентилятора испарителя
//    uint8_t countErr;               //счётчик аварий отказа вентилятора испарителя
    uint8_t setPercentPower;
} FANEvaporatorTypeDef;
extern FANEvaporatorTypeDef FanEvaporatorStr;	//параметры вентилятора испарителя

#endif /* __TD_DATA_H */

