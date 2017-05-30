#include "f_flash.h"


//*****************************************************************************
//Работа с FLASH памятью
//*****************************************************************************
//Функция стирания одной страницы
//pageAddress - любой адрес, принадлежащий стираемой странице
void Flash_Erase(unsigned int pageAddress)
{
    while (FLASH->SR & FLASH_SR_BSY);
    if (FLASH->SR & FLASH_SR_EOP) {
        FLASH->SR = FLASH_SR_EOP;
    }

    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = pageAddress;
    FLASH->CR |= FLASH_CR_STRT;
    while (!(FLASH->SR & FLASH_SR_EOP));
    FLASH->SR = FLASH_SR_EOP;
    FLASH->CR &= ~FLASH_CR_PER;
}


//Функция записи
//data - указатель на записываемые данные
//address - адрес во FLASH
//count - количество записываемых байт, должно быть кратно 2
void Flash_Write(unsigned char* data, unsigned int address, unsigned int count)
{
    unsigned int i;

    while (FLASH->SR & FLASH_SR_BSY);
    if (FLASH->SR & FLASH_SR_EOP) {
        FLASH->SR = FLASH_SR_EOP;
    }

    FLASH->CR |= FLASH_CR_PG;

    for (i = 0; i < count; i += 2) {
        *(volatile unsigned short*)(address + i) = (((unsigned short)data[i + 1]) << 8) + data[i];
        while (!(FLASH->SR & FLASH_SR_EOP));
        FLASH->SR = FLASH_SR_EOP;
    }

    FLASH->CR &= ~(FLASH_CR_PG);
}


//Функция чтения FLASH
//address - адрес во FLASH
//return - считываемые данные
uint32_t Flash_Read(uint32_t address) 
{
	return (*(__IO uint32_t*) address);
}



//Чтение параметрируемых значений из FLASH
void read_flash_value(void)
{
	*(__IO uint32_t*)&res_table.regsF3_6[20] = Flash_Read(ADDRESS_PAGE_63);
	*(__IO uint32_t*)&res_table.regsF3_6[22] = Flash_Read(ADDRESS_PAGE_63+4);
	*(__IO uint32_t*)&res_table.regsF3_6[24] = Flash_Read(ADDRESS_PAGE_63+8);
	*(__IO uint32_t*)&res_table.regsF3_6[26] = Flash_Read(ADDRESS_PAGE_63+12);
	*(__IO uint32_t*)&res_table.regsF3_6[28] = Flash_Read(ADDRESS_PAGE_63+16);
	*(__IO uint32_t*)&res_table.regsF3_6[30] = Flash_Read(ADDRESS_PAGE_63+20);
	*(__IO uint32_t*)&res_table.regsF3_6[32] = Flash_Read(ADDRESS_PAGE_63+24);
	*(__IO uint32_t*)&res_table.regsF3_6[34] = Flash_Read(ADDRESS_PAGE_63+28);
	*(__IO uint32_t*)&res_table.regsF3_6[36] = Flash_Read(ADDRESS_PAGE_63+32);
	*(__IO uint32_t*)&res_table.regsF3_6[38] = Flash_Read(ADDRESS_PAGE_63+36);
	*(__IO uint32_t*)&res_table.regsF3_6[40] = Flash_Read(ADDRESS_PAGE_63+40);
    *(__IO uint32_t*)&res_table.regsF3_6[42] = Flash_Read(ADDRESS_PAGE_63+44);
    *(__IO uint32_t*)&res_table.regsF3_6[44] = Flash_Read(ADDRESS_PAGE_63+48);
    *(__IO uint32_t*)&res_table.regsF3_6[46] = Flash_Read(ADDRESS_PAGE_63+52);
    *(__IO uint32_t*)&res_table.regsF3_6[48] = Flash_Read(ADDRESS_PAGE_63+56);
}


//Запись параметрируемых значений на FLASH
void write_flash_value(void)
{
	FLASH_Unlock();
	Flash_Erase(ADDRESS_PAGE_63); 	//стираем 63 страницу в памяти
	Flash_Write((uint8_t*)&res_table.regsF3_6[20], ADDRESS_PAGE_63, 60); //Сохраняем на FLASH
	FLASH_Lock();
}
