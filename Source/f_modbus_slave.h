/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __F_MODBUS_SLAVE_H
#define __F_MODBUS_SLAVE_H

#include "main_ini.h"
#include <string.h>


void modbus_slave1(UartDataTypeDef *MODBUS);	//функция обработки ModBus и формирования ответа ВНУТРЕННИЙ
void modbus_slave2(UartDataTypeDef *MODBUS);	//функция обработки ModBus и формирования ответа ВНЕШНИЙ

void TX_01(UartDataTypeDef *MODBUS);
void TX_03_04(UartDataTypeDef *MODBUS);
void TX_05(UartDataTypeDef *MODBUS);
void TX_06(UartDataTypeDef *MODBUS);
void TX_43(UartDataTypeDef *MODBUS);
void TX_EXCEPTION(UartDataTypeDef *MODBUS,unsigned char error_type);
void saveValueTX06(uint8_t tmp, UartDataTypeDef *MODBUS);

uint32_t Crc16(uint8_t *ptrByte, uint32_t byte_cnt);


#endif /* __F_MODBUS_SLAVE_H */
