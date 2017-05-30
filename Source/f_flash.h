/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __F_FLASH_H
#define __F_FLASH_H

#include "main_ini.h"


void Flash_Erase(unsigned int pageAddress);
void Flash_Write(unsigned char* data, unsigned int address, unsigned int count);
uint32_t Flash_Read(uint32_t address);
void read_flash_value(void);
void write_flash_value(void);


#endif /* __F_FLASH_H */
