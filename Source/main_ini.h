/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_INI_H
#define __MAIN_INI_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_adc.h"
//#include "stm32f0xx_i2c.h"
//#include "i2c.h"

#include "td_data.h"
#include "f_ds18b20.h"
#include "f_delays.h"
#include "f_air_control.h"
#include "f_air_alarm.h"
#include "f_flash.h"
#include "f_modbus_slave.h"


void pin_IN(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void pin_OUT_OD(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void pin_OUT_PP(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

void initializeCompress(void);
void initializeADC(void);
void initializeFans(void);
void initializeTachoFans(void);
void initializeRelays(void);
void initializeLeds(void);
void initializeSensorsTemp(void);
void initializeTimerDelay(void);
void initializeTimerTachoCompres(void);
void initializeUSART1ModBusInt(void);
void initializeUSART2ModBusExt(void);
void initializeTimerModBusTimeout(void);


#endif /* __MAIN_INI_H */
