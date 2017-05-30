#include "f_modbus_slave.h"

//Значения скоростей для ModBus
uint32_t numSpeed[] = { 2400,       //0 
                        4800,       //1
                        7200,       //2
                        9600,       //3
                        14400,      //4
                        19200,      //5
                        38400,      //6
                        57600, 		//7
                        115200};	//8


//*********************************************************************
//Modbus slave function ВНУТРЕННИЙ
//*********************************************************************
void modbus_slave1(UartDataTypeDef *MODBUS)
{
  unsigned int tmp;
	
     //recive and checking rx query
	if((MODBUS->buffer[0]!=0) & (MODBUS->rxcnt>5) & ((MODBUS->buffer[0]==modBusInt.adress) | (MODBUS->buffer[0]==255)))
 {
		tmp=Crc16(MODBUS->buffer,MODBUS->rxcnt-2);

		if((MODBUS->buffer[MODBUS->rxcnt-2]==(tmp&0x00FF)) & (MODBUS->buffer[MODBUS->rxcnt-1]==(tmp>>8)))
    {
			//choosing function
			switch(MODBUS->buffer[1])
			{
//				case 1:
//				TX_01(MODBUS);
//				break;
				
				case 3:
				TX_03_04(MODBUS);
				break;

				case 4:
				TX_03_04(MODBUS);
				break;
				
				case 5:
				TX_05(MODBUS);
				break;

				case 6:
				TX_06(MODBUS);
				break;
				
				case 43:
				TX_43(MODBUS);
				break;

				default:
				//illegal operation
				TX_EXCEPTION(MODBUS,0x01);
                break;
			}
				//adding CRC16 to reply
				tmp=Crc16(MODBUS->buffer,MODBUS->txlen-2);
				MODBUS->buffer[MODBUS->txlen-2]=tmp;
				MODBUS->buffer[MODBUS->txlen-1]=tmp>>8;
				MODBUS->txcnt=0;
		}
 }
	  MODBUS->rxcnt=0;
	  MODBUS->rxtimer=0xFFFF;
}


//*********************************************************************
//Modbus slave function ВНЕШНИЙ
//*********************************************************************
void modbus_slave2(UartDataTypeDef *MODBUS)
{
  unsigned int tmp;

     //recive and checking rx query
	if((MODBUS->buffer[0]!=0) & (MODBUS->rxcnt>5) & ((MODBUS->buffer[0]==modBusExt.adress) | (MODBUS->buffer[0]==255)))
 {
		tmp=Crc16(MODBUS->buffer,MODBUS->rxcnt-2);

		if((MODBUS->buffer[MODBUS->rxcnt-2]==(tmp&0x00FF)) & (MODBUS->buffer[MODBUS->rxcnt-1]==(tmp>>8)))
    {
			//choosing function
			switch(MODBUS->buffer[1])
			{
//				case 1:
//				TX_01(MODBUS);
//				break;
				
				case 3:
				TX_03_04(MODBUS);
				break;

				case 4:
				TX_03_04(MODBUS);
				break;
				
				case 5:
				TX_05(MODBUS);
				break;

				case 6:
				TX_06(MODBUS);
				break;
				
				case 43:
				TX_43(MODBUS);
				break;

				default:
				//illegal operation
				TX_EXCEPTION(MODBUS,0x01);
                break;
			}
				//adding CRC16 to reply
				tmp=Crc16(MODBUS->buffer,MODBUS->txlen-2);
				MODBUS->buffer[MODBUS->txlen-2]=tmp;
				MODBUS->buffer[MODBUS->txlen-1]=tmp>>8;
				MODBUS->txcnt=0;
		}
 }
	  MODBUS->rxcnt=0;
	  MODBUS->rxtimer=0xFFFF;
}



//*******************************************************
//Read Coils
//*******************************************************
//inline void TX_01(typeDef_UART_DATA *MODBUS)
//{
//  unsigned int tmp;
//	unsigned int tmp1;

//	unsigned char m=0, n=0;
//  //MODBUS[0] =uartX.adress; // adress - stays a same as in recived
//  //MODBUS[1] = 1; //query type - - stay a same as in recived
//  
//  //2-3  - adress   , 4-5 - Quantity of Outputs
//   
//  tmp = ((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); 	//Adress
//	tmp1 = ((MODBUS->buffer[4]<<8)+MODBUS->buffer[5]); 	//Quantity of Outputs
//	
//	
//	//Read from 1 to 2000 contiguous status of coils device
//	if(tmp1>=0x0001 || tmp1<=0x07D0){
//		if((tmp <= OBJ_SZ_F1_5*8) && ((tmp+tmp1) <= OBJ_SZ_F1_5*8)){
//			//Byte Count
//			if(tmp1%8!=0){
//				MODBUS->buffer[2] = (tmp1/8)+1;
//			}
//			else{
//				MODBUS->buffer[2] = (tmp1/8);
//			}
//			MODBUS->txlen = MODBUS->buffer[2]+5;
//			
//			MODBUS->buffer[3] = res_table.regsF1_5[0] >> (tmp1%8);
//			MODBUS->buffer[3] = res_table.regsF1_5[1] << (8-(tmp1%8)) & MODBUS->buffer[3];
//			MODBUS->buffer[4] = res_table.regsF1_5[1] >> (tmp1%8);
//		}
//		else{
//			//illegal data
//			TX_EXCEPTION(MODBUS,0x02);
//		}
//	}
//	else{
//		//illegal data
//		TX_EXCEPTION(MODBUS,0x03);
//	}
//}


//******************************************************************
//READING input & holding registers
//*******************************************************************
void TX_03_04(UartDataTypeDef *MODBUS)
{
	unsigned int tmp,tmp1;
	unsigned int m=0,n=0;
	int tmp_val,tmp_val_pos;
		
	//MODBUS->buffer[0] = uartX.adress; // adress - stays a same as in received
	//MODBUS->buffer[1] = 3; //query type - - stay a same as in recived
	//MODBUS->buffer[2] = data byte count

	//2-3  - starting address
	tmp = ((MODBUS->buffer[2]<<8) + MODBUS->buffer[3]); 

	//4-5 - number of registers
	tmp1 = ((MODBUS->buffer[4]<<8) | MODBUS->buffer[5]);

	//default answer length if error
	n=3; 

	if((((tmp+tmp1)<OBJ_SZ_F3_6) & (tmp1<MODBUS_WRD_SZ+1))){
		for(m=0;m<tmp1;m++){
			tmp_val=res_table.regsF3_6[m+tmp];
			if(tmp_val<0){
				tmp_val_pos=tmp_val;
				MODBUS->buffer[n]=(tmp_val_pos>>8)|0x80;
				MODBUS->buffer[n+1]=tmp_val_pos;
			}
			else{
				MODBUS->buffer[n]=tmp_val>>8;
				MODBUS->buffer[n+1]=tmp_val;
			}
			n=n+2;
    }
		MODBUS->buffer[2]=m*2; //byte count
		MODBUS->txlen=m*2+5; 	//responce length
  }
  else{
		//exception illegal data adress 0x02
		TX_EXCEPTION(MODBUS,0x02);
  }
 
}


//*******************************************************
//Writing one bit
//*******************************************************
void TX_05(UartDataTypeDef *MODBUS)
{
  unsigned int tmp;
	unsigned int tmp1;
	//unsigned char n;
  //MODBUS[0] = uartX.adress; // adress - stays a same as in recived
  //MODBUS[1] = 5; //query type - - stay a same as in recived
  //2-3  - adress   , 4-5 - value
  tmp = ((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); 	//Adress
	tmp1 = ((MODBUS->buffer[4]<<8)+MODBUS->buffer[5]); 	//Output Value

	//n = tmp/8;

	if(tmp1==0x0000 || tmp1==0xFF00){
		//MODBUS->buffer[2]  - byte count a same as in rx query
		if(tmp<OBJ_SZ_F1_5*8){
			MODBUS->txlen=MODBUS->rxcnt; //responce length
				if(tmp1==0x0000){
					res_table.regsF1_5[tmp/8]&=~(1<<(tmp%8));
				}
				else{
					res_table.regsF1_5[tmp/8]|=(1<<(tmp%8));
				}		
		}
		else{
			//illegal data
			TX_EXCEPTION(MODBUS,0x02);
		}
	}
	else{
		//illegal data
		TX_EXCEPTION(MODBUS,0x03);
	}
}


// дополнительная функция для TX_06: сохраняет значение в карту регистров
void saveValueTX06(uint8_t tmp, UartDataTypeDef *MODBUS)
{
    //MODBUS->buffer[2]  - byte count a same as in rx query
    if(tmp<OBJ_SZ_F3_6){
        MODBUS->txlen=MODBUS->rxcnt; //responce length
        res_table.regsF3_6[tmp]=(MODBUS->buffer[4]<<8)+MODBUS->buffer[5];
    } else {
    //illegal data
        TX_EXCEPTION(MODBUS,0x02);
    }
}


//*******************************************************
//Writing register
//*******************************************************
void TX_06(UartDataTypeDef *MODBUS)
{
    unsigned int tmp;
    unsigned int bufReg = 0;
    int16_t buf = 0;
  //MODBUS[0] = uartX.adress; // adress - stays a same as in recived
  //MODBUS[1] = 6; //query type - - stay a same as in recived

  //2-3  - adress   , 4-5 - value

  tmp =((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //adress

	if(tmp < 20){
		TX_EXCEPTION(MODBUS,0x02);
		return;
	}
	
	switch (tmp){
/***************************************************************************/
		/*  Значение задания рабочей температуры внутри шкафа*/
		case 20:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 40) || (MODBUS->buffer[5] < 10)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания Δt температуры внутри шкафа*/
		case 21:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 20)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания температуры внутри шкафа для срочного включения компрессора*/
		case 22:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 90) || (MODBUS->buffer[5] <= W_T_INT)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания аварийно-высокой температуры внитри шкафа*/
		case 23:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 90) || (MODBUS->buffer[5] < W_TQON)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания температуры внутри шкафа для срочного выключения компрессора*/
		case 24:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] >= (W_T_INT - W_DT_INT)) || (MODBUS->buffer[5] < 5)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания аварийно-низкой температуры внитри шкафа*/
		case 25:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] >= W_TQOFF) || (MODBUS->buffer[5] < 5)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания сетевого адреса ModBus (1 - 127)*/
		case 26:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 127) || (MODBUS->buffer[5] < 1)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;			

/***************************************************************************/
		/*  Значение задания скорости обмена*/
		case 27:
			//если задаваемое значение больше 5
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 8)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания аварийной температуры хладагента*/
		case 28:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 90) || (MODBUS->buffer[5] < 30)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания Δt аварийной температуры хладагента*/
		case 29:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 30)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания температуры для зоны «+» конденсатора*/
		case 30:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 50) || (MODBUS->buffer[5] < 20) 
            || (MODBUS->buffer[5] >= W_TZPPK) || (MODBUS->buffer[5] < W_TZMK)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания температуры для зоны «++» конденсатора*/
		case 31:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 50) || (MODBUS->buffer[5] < 20) 
            || (MODBUS->buffer[5] <= W_TZPK)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания температуры для зоны «-» конденсатора*/
		case 32:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 50) || (MODBUS->buffer[5] < 15)
            || (MODBUS->buffer[5] <= W_TZMMK) || (MODBUS->buffer[5] > W_TZPK)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания температуры для зоны «--» конденсатора*/
		case 33:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 50) || (MODBUS->buffer[5] < 15)
            || (MODBUS->buffer[5] >= W_TZMK)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания интервала времени для зоны «+» конденсатора*/
		case 34:
			bufReg = ((bufReg | MODBUS->buffer[4]) << 8) | MODBUS->buffer[5]; //Объединение байтов регистров
			if ((bufReg > 500) || (bufReg < 1)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания интервала времени для зоны «++» конденсатора*/
		case 35:
			bufReg = ((bufReg | MODBUS->buffer[4]) << 8) | MODBUS->buffer[5]; //Объединение байтов регистров
			if ((bufReg > 500) || (bufReg < 1)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания интервала времени для зоны «-» конденсатора*/
		case 36:
			bufReg = ((bufReg | MODBUS->buffer[4]) << 8) | MODBUS->buffer[5]; //Объединение байтов регистров
			if ((bufReg > 500) || (bufReg < 1)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Значение задания температуры для включения фрикулига*/
		case 37:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 50)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
			/*  Значение задания гистерезиса для фрикулинга*/
		case 38:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 20)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;
/***************************************************************************/
			/*  Значение задания температуры обмерзания испарителя (t6)*/
		case 39:
            buf = 0;
            buf = ((buf | MODBUS->buffer[4]) << 8) | MODBUS->buffer[5]; //Объединение байтов регистров
			if ((buf < -10) || (buf > 0)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;
/***************************************************************************/
			/*  Значение задания гистерезиса обмерзания испарителя*/
		case 40:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 20)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;
/***************************************************************************/
           /*  Значение задания аварийно-высокого напряжения питания*/
		case 41:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 60) || (MODBUS->buffer[5] <= W_ALARM_POWER_L)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;
            
/***************************************************************************/
            /*  Значение задания аварийно-низкого напряжения питания*/
		case 42:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] >= W_ALARM_POWER_H) || (MODBUS->buffer[5] < 36)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;
/***************************************************************************/
            /*  Значение задания аварийной температуры компрессора*/
		case 43:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 120) || (MODBUS->buffer[5] < 60)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;
/***************************************************************************/
            /*  Значение задания минимальной температуры включения компрессора*/
		case 44:
            buf = 0;
            buf = ((buf | MODBUS->buffer[4]) << 8) | MODBUS->buffer[5]; //Объединение байтов регистров
			if ((buf < -10) || (buf > 10)) {
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;
/***************************************************************************/            
		/*  Задание мощности вентилятора конденсатора (для режима «ТЕСТ»)*/
		case 45:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 100)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
		/*  Задание мощности вентилятора испарителя (для режима «ТЕСТ»)*/
		case 46:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 100)){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;
            
/***************************************************************************/
            /*  Задание мощности компрессора (для режима «ТЕСТ»)*/
		case 47:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 100) 
            || ((MODBUS->buffer[5] < MIN_P_COMPRES) && (MODBUS->buffer[5] > 0))){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;

/***************************************************************************/
            /*  Задание MAX. RPM компрессора */
		case 48:
			buf = 0;
			buf = ((buf | MODBUS->buffer[4]) << 8) | MODBUS->buffer[5]; //Объединение байтов регистров
			if (buf < 1000 || buf > 6000){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;
            
/***************************************************************************/
      /*  Задание температуры включения ТЕНа */
		case 49:
			if ((MODBUS->buffer[4] != 0) || (MODBUS->buffer[5] > 25)
            || ((MODBUS->buffer[5] < MIN_P_COMPRES) && (MODBUS->buffer[5] > 0))){
				TX_EXCEPTION(MODBUS,0x03);	//выводим ошибку
			} else {
				saveValueTX06(tmp, MODBUS);
			}
			break;
            
/***************************************************************************/
 	
		default:
			//MODBUS->buffer[2]  - byte count a same as in rx query
			if (tmp<OBJ_SZ_F3_6){
				MODBUS->txlen=MODBUS->rxcnt; //responce length
				res_table.regsF3_6[tmp]=(MODBUS->buffer[4]<<8)+MODBUS->buffer[5];
			} else {
				//illegal data
				TX_EXCEPTION(MODBUS,0x02);
			}
			break;
	}
}


//*******************************************************
//Read identification
//*******************************************************
void TX_43(UartDataTypeDef *MODBUS)
{
  //MODBUS[0] = uartX.adress; //adress - stays a same as in recived
  //MODBUS[1] = 43; 					//query type - stay a same as in recived
	//MODBUS[2] = 0x0E; 				//MEI Type  - stay a same as in recived
	//MODBUS[3] = 0x01; 				//Read Dev Id code  - stay a same as in recived
  
	
	if(MODBUS->buffer[4] == 0x00)		//If Object Id OK
	{
		//If Read deviceId Code OK
		if(MODBUS->buffer[3] == 0x01){	
			MODBUS->txlen = 32; 										//responce length
			MODBUS->buffer[4]	= 0x01;								//Conformity Level 
			MODBUS->buffer[5]	= 0x00;								//More Follows
			MODBUS->buffer[6]	= 0x00;								//NextObjectId
			MODBUS->buffer[7]	= 0x03;								//Number Of Objects
			MODBUS->buffer[8]	= 0x00;								//Object Id
			MODBUS->buffer[9]	= 4;									//Object Length
			MODBUS->buffer[10]= CompanyID[0];				//Object Value	"*"
			MODBUS->buffer[11]= CompanyID[1];				//Object Value	"*"
			MODBUS->buffer[12]= CompanyID[2];				//Object Value	"*"
			MODBUS->buffer[13]= CompanyID[3];				//Object Value	"*"
			MODBUS->buffer[14]= 0x01;								//Object Id
			MODBUS->buffer[15]= 7;									//Object Length
			MODBUS->buffer[16]= ProdCode[0];				//Object Value	"A"
			MODBUS->buffer[17]= ProdCode[1];				//Object Value	"i"
			MODBUS->buffer[18]= ProdCode[2];				//Object Value	"r"
			MODBUS->buffer[19]= ProdCode[3];				//Object Value	"c"
			MODBUS->buffer[20]= ProdCode[4];				//Object Value	"o"
			MODBUS->buffer[21]= ProdCode[5];				//Object Value	"o"
			MODBUS->buffer[22]= ProdCode[6];				//Object Value	"l"
			MODBUS->buffer[23]= 0x02;								//Object Id
			MODBUS->buffer[24]= 5;									//Object Length
			MODBUS->buffer[25]= Version[0];					//Object Value	"x"
			MODBUS->buffer[26]= Version[1];					//Object Value	"."
			MODBUS->buffer[27]= Version[2];					//Object Value	"x"
			MODBUS->buffer[28]= Version[3];					//Object Value	"."
			MODBUS->buffer[29]= Version[4];					//Object Value	"x"
			MODBUS->buffer[30]= Version[5];					//Object Value	"x"
			MODBUS->buffer[31]= Version[6];					//Object Value	"x"
			MODBUS->buffer[32]= Version[7];					//Object Value	"x"
			MODBUS->buffer[33]= Version[8];					//Object Value	"x"
			MODBUS->buffer[34]= Version[9];					//Object Value	"x"
			MODBUS->buffer[35]= Version[10];				//Object Value	"x"
		}
		else{
			//illegal data
			TX_EXCEPTION(MODBUS,0x03);
		}
	}
	else{
		//illegal data
		TX_EXCEPTION(MODBUS,0x02);
	}
}



//********************************************************************
//Exception if wrong query
//*********************************************************************

//modbus exception - illegal data=01 ,adress=02 etc 
void TX_EXCEPTION(UartDataTypeDef *MODBUS,unsigned char error_type)
{
	//illegal operation	 
	MODBUS->buffer[1] = MODBUS->buffer[1]|0x80;
	MODBUS->buffer[2] = error_type; //exception
	MODBUS->txlen = 5; //responce length  
}

//*********************************************************************
//CRC16 for Modbus Calculation
//*********************************************************************

uint32_t Crc16(uint8_t *ptrByte, uint32_t byte_cnt)
{
	uint32_t w=0;
	uint8_t shift_cnt;

	if(ptrByte){
		w=0xffffU;
		for(; byte_cnt>0; byte_cnt--){
			w=(uint32_t)((w/256U)*256U+((w%256U)^(*ptrByte++)));
			for(shift_cnt=0; shift_cnt<8; shift_cnt++){
				if((w&0x1)==1){
					w=(uint32_t)((w>>1)^0xa001U);
				}
				else{
					w>>=1;
				}
			}
		}
	}
	return w;
}

/*
unsigned int Crc16 (unsigned char *ptrByte, int byte_cnt)
{
    static const int wCRCTable[] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

    unsigned char nTemp;
    int wCRCWord = 0xFFFF;

    while (byte_cnt--)
    {
        nTemp = *ptrByte++ ^ wCRCWord;
        wCRCWord >>= 8;
        wCRCWord  ^= wCRCTable[(nTemp & 0xFF)];
    }
    return wCRCWord;
} // End: CRC16
*/

