/* Includes ------------------------------------------------------------------*/
#include "f_ds18b20.h"
#include "main_ini.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t dowcrc;              //для проверки CRC
static uint8_t countErrCRC;
static uint8_t countErrConnect;

const uint8_t dscrc_table[] = {0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
      157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
       35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
      190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
       70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
      219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
      101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
      248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
      140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
       17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
      175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
       50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
      202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
       87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
      233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
      116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};



//Функция производит первоначальную настройку порта и датчика DS18B20
uint8_t one_wire_ini_setting(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    uint8_t stat = 0;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    //Кофигурируем пин
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(GPIOx, &GPIO_InitStruct);
    
    //Первоначальное конфигурирование датчика DS18B20
    stat = send_presence(GPIOx, GPIO_Pin);
    one_wire_write_byte(GPIOx, GPIO_Pin, THERM_CMD_SKIPROM);        //обращаемся к единственному устройству на шине
    one_wire_write_byte(GPIOx, GPIO_Pin, THERM_CMD_WSCRATCHPAD);    //запись в регистры
    one_wire_write_byte(GPIOx, GPIO_Pin, 0x7D);                     //верхний порог тревоги +125
    one_wire_write_byte(GPIOx, GPIO_Pin, 0xB2);                     //нижний порог тревоги -50
    one_wire_write_byte(GPIOx, GPIO_Pin, 0x1F);                     //разрядность 9 бит
    return stat;
}


//Функция выдаёт сигнал сброса для инициализации One-Wire (сигнал presence)
uint8_t send_presence(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) 
{
    uint8_t stat = 0;
    if ((GPIOx->IDR&GPIO_Pin)==0) return 2;     //Проверка замыкания на шину земли
    pin_OUT_OD(GPIOx, GPIO_Pin);
    GPIOx->BRR = GPIO_Pin;                      //Притягиваем шину к земле
    delay_mks(480);                             //Ждём минимум 480us
    GPIOx->BSRR = GPIO_Pin;                     //Отпускаем шину
    pin_IN(GPIOx, GPIO_Pin);
    delay_mks(60);                              //Ждём 30-100us
    stat = (GPIOx->IDR&GPIO_Pin ? 1:0);         //Проверка присутствия датчика
    delay_mks(420);                             //Ждём 420us
    return stat;                                //Возвращаем состояние (0) ok, (-1) на шине нет датчика
}


//Функция ЗАПИСИ одного бита
void one_wire_write_bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t bit)
{
    pin_OUT_OD(GPIOx, GPIO_Pin);                //Настраиваем пин на выход
    GPIOx->BRR = GPIO_Pin;                      //Притягиваем шину к "0"
    delay_mks(3);                               //Ждём 3us
    if(bit){GPIOx->BSRR = GPIO_Pin;}            //Если хотим отправить "1", отпускаем шину
    delay_mks(80);                              //Ждём от 60us до 120us
    GPIOx->BSRR = GPIO_Pin;                     //Отпускаем шину
}


//Функция ЧТЕНИЯ одного бита
uint8_t one_wire_read_bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    uint8_t bit = 0;
    pin_OUT_OD(GPIOx, GPIO_Pin);                //Настраиваем пин на выход
    GPIOx->BRR = GPIO_Pin;                      //Притягиваем шину к "0"
    delay_mks(1);                               //Ждём 1us 
    GPIOx->BSRR = GPIO_Pin;                     //Отпускаем шину
    delay_mks(1);                               //Ждём 1us
    pin_IN(GPIOx, GPIO_Pin);                    //Настраиваем пин на вход
    bit = (GPIOx->IDR&GPIO_Pin?1:0);            //Читаем бит
    delay_mks(45);                              //Ждём 45us 
    return bit;                                 //Возвращаем прочитанный бит
}

//Функция ЧТЕНИЯ одного байта
uint8_t read_byte(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    uint8_t i;
    uint8_t value = 0;

    for (i=0; i<8; i++) {
        if(one_wire_read_bit(GPIOx, GPIO_Pin)) { value|=0x01<<i; }
        //delay_mks(120);
    }
    return(value);
}


//Функция ЗАПИСИ одного байта (8 бит)
void one_wire_write_byte(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t data)
{
    uint8_t i;
    for (i = 0; i<8; i++) {		
        one_wire_write_bit(GPIOx, GPIO_Pin, data>>i & 1);
    }
}


//Функция запускает измерение температуры
void one_ware_convert_t(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
		send_presence(GPIOx, GPIO_Pin);                                 //Сброс
		one_wire_write_byte(GPIOx, GPIO_Pin, THERM_CMD_SKIPROM);        //Обращаемся к единственному устройству на шине
		one_wire_write_byte(GPIOx, GPIO_Pin, THERM_CMD_CONVERTTEMP);	//Запускаем измерение
}	


//Функция ЧТЕНИЯ одного байта с проверкой CRC(8 бит)
//uint16_t one_wire_read_byte(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
//{
//	uint8_t stat = 0;
//	uint8_t Nbyte = 0;
//	uint8_t arr[9];	
//	uint16_t dataT = 0;
//	dowcrc = 0;
//	
//	stat = send_presence(GPIOx, GPIO_Pin);          //Сброс датчика
//	
//	if(!stat){																											//Проверка шины на ошибки
//		one_wire_write_byte(GPIOx, GPIO_Pin, THERM_CMD_SKIPROM);        //Обращаемся к единственному устройству на шине
//		one_wire_write_byte(GPIOx, GPIO_Pin, THERM_CMD_RSCRATCHPAD);    //Чтение регистров
//		for(Nbyte = 0; Nbyte<9; Nbyte++)                                //Чтение 9 байт из RAM
//		{
//			arr[Nbyte] = read_byte(GPIOx, GPIO_Pin);
//			if(Nbyte<8){
//				ow_crc(arr[Nbyte]);                 //Проверка CRC
//			}
//		}
//		*((uint8_t *)&dataT) = arr[0];              //Извлекаем младший байт температуры
//		*((uint8_t *)&dataT+1) = arr[1];            //Извлекаем старший байт и объединеняем с младшим
//		dataT = dataT >> 4;                         //Отбрасываем значение после запятой					
//		
//		//Если CRC неверно
//		if(dowcrc != arr[8]){		
//			countErrCRC++;
//			if(countErrCRC < 10){                   //если счётчик ошибок меньше 10
//				return one_wire_read_byte(GPIOx, GPIO_Pin);	//повторяем чтение
//			}
//			else{
//				countErrCRC = 0;
//				return 0x400;                       //возвращаем ошибку CRC (поломка датчика)
//			}
//		}
//		countErrCRC = 0;
//		
//		
//		if(dataT & 0x80){                           //Если температура отрицательна
//			return  (uint8_t)((~dataT)|0x0080);     //Возвращаем значение в виде 0b1xxxxxxx
//		}
//		return  dataT;                              //Возвращаем полученное значение в виде 0b0xxxxxxx
//	}
//	else{
//		countErrConnect++;
//		if(countErrConnect < 3){
//			return one_wire_read_byte(GPIOx, GPIO_Pin);	//повторяем чтение
//		}
//		else{
//			countErrConnect = 0;
//			return (stat << 8);                     //Возвращаем ошибку если на шине нет датчика или замыкание
//		}
//	}
//}



/**
* @brief    Функция вычитывает значения температуры датчика ds18b20.
* @param    GPIOx: здесь x это буква порта переферии (A, B, C, D, E or F).
* @param    GPIO_Pin: определяет бит порта для чтения.
* @note     Данная функция возвращает значение температуры датчика DS18B20 с точностью 9 бит (0,5°C). 
            Значение температуры конвертируется в дополнительный код и умножается на 10. 
            В случае обрыва, замыкания на 0 или неправильной CRC функция возвращает код ошибки.
            Полученное значение кратно 5.
            т.е. значение 215 = 0xD7 = +21,5°C
                            5 = 0x05 = +0,5°C
                           -5 = 0xFFFB = -0,5°C
                         -215 = 0xFF29 = -21,5°C							
* @retval   valueTemp: значение температуры в дополнительном коде умноженное на 10.
            если датчик не подключен - возвращает 0xFFFF (-1).
            statErr: код ошибки:    0xEFFF – короткое замыкание шины данных на землю
                                    0xDFFF – ошибка CRC
*/
int16_t one_wire_read_byte_dec(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    uint8_t dowCRC = 0;         /*!< для проверки CRC */
    int16_t statErr = 0;        /*!< статус ошибки */
    uint8_t numberByte = 0;     /*!< номер байта для чтения */
    uint8_t dataArr[9];         /*!< массив считанных с датчика данных */
    int16_t valueTemp = 0;      /*!< значение температуры */
    int16_t temp = 0;           /*!< буфер  */

    statErr = send_presence(GPIOx, GPIO_Pin);                           /*!< Сброс датчика  (сигнал presence) */
    /* Если нет ошибок датчика*/
    if (!statErr) {
        one_wire_write_byte(GPIOx, GPIO_Pin, THERM_CMD_SKIPROM);        /*!< Обращаемся к единственному устройству на шине */
        one_wire_write_byte(GPIOx, GPIO_Pin, THERM_CMD_RSCRATCHPAD);    /*!< Чтение регистров */
        for(numberByte = 0; numberByte < 9; numberByte++)               /*!< Чтение 9 байт из RAM */
        {
            dataArr[numberByte] = read_byte(GPIOx, GPIO_Pin);
            if (numberByte<8) {
                dowCRC = dscrc_table[dowCRC^dataArr[numberByte]];       /*!< Вычисление CRC */
            }
        }
        /* Проверка CRC */
        if (dowCRC != dataArr[8]) {		
            countErrCRC++;
            /* если счётчик ошибок меньше 10 */
            if (countErrCRC < 10) {											 		
                return one_wire_read_byte_dec(GPIOx, GPIO_Pin);             /*!< повторяем чтение */
            } else {
                countErrCRC = 0;
                return ERROR_CRC;                                       /*!< возвращаем ошибку CRC (поломка датчика) */
            }
        }
        countErrCRC = 0;

        *((uint8_t *)&temp) = dataArr[0];           /*!< Извлекаем младший байт температуры */
        *((uint8_t *)&temp+1) = dataArr[1];         /*!< Извлекаем старший байт и объединеняем с младшим */
        temp = temp >> 4;                           /*!< Копируем целую часть в буффер */
        /* Если температура отрицательная */
        if (dataArr[1] & 0x80) {
            /* если значение после запятой равно 0,5 */
            if(!(dataArr[0] & 0x08)){
                valueTemp = ~((~(temp|0xF000)) * 10 + 5)+1; /*!< Преобразуем значение в дополнительный код и умножаем на 10 */
            } else {
                valueTemp = ~((~(temp|0xF000)) * 10)+1;     /*!< Преобразуем значение в дополнительный код и умножаем на 10 */
            }
        }
        /* иначе если температура положительная */
        else {
            /* если значение после запятой равно 0,5 */
            if (dataArr[0] & 0x08) {
                valueTemp = temp * 10 + 5;          /*!< Значение умножаем на 10 */
            } else {
                valueTemp = temp * 10;              /*!< Значение умножаем на 10 */
            }
        }
        return  valueTemp;                          /*!< Возвращаем полученное значение */
    } else {
        countErrConnect++;
		if(countErrConnect < 10){
			return one_wire_read_byte_dec(GPIOx, GPIO_Pin);	//повторяем чтение
		}
		else{
			countErrConnect = 0;
			/*!< Возвращаем ошибку */
            if (statErr == 2){
                return ERROR_SC_GND;
            } else {
                return ERROR_NO_SENSOR;
            }
        }
        
    }
}



//Функция вычисления контрольной суммы crc8 для DS18B20
uint8_t ow_crc(uint8_t x)
{
	dowcrc = dscrc_table[dowcrc^x];
return dowcrc;
}


