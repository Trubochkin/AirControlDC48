/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COND_OPER_H
#define __COND_OPER_H

#include "main_ini.h"

uint8_t getErrorSensor(int16_t valueSensor);
int16_t getValueSensor(int16_t valueSensor);


void funcRunDelayed(uint8_t timerNum, uint32_t setTime);

void runOnce1s (void);
void updateCompressorState (void);
void updateSpeedFAN_C (void);
void updateSpeedFAN_I (void);
void changeModbusAddressSpeed (void);
void blinkLedStat (void);
void blinkLedAlarm (void);
void computeRpmCompress (void);
void computeRpmFanC (void);
void computeRpmFanI (void);
void computeTempVal (void);
uint16_t convertTemperature (uint16_t value);
void computeValueADC (void);
void controlCompress (void);
void controlTen (void);
void controlFanC (void);
void controlfanE (void);
void controlHand (void);

#endif
