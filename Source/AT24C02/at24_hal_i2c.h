/*
 * at24_hal_i2c.h
 *
 *  Created on: Sep,10,2017
 *      Author: Amir Parto
 */




#ifndef DRIVERS_MYLIB_AT24_HAL_I2C_H_
#define DRIVERS_MYLIB_AT24_HAL_I2C_H_

#define AT24C_ADDRESS   						0xA0
#define AT24C_I2C_Handle  					hi2c1  
#define AT24C64											64
#define AT24C32											32
#define AT24C02											02
#define AT24C512										512




#define AT24Cxx											AT24C02



#if AT24Cxx == AT24C512
	#define AT24c_I2C_MEMADD_SIZE				I2C_MEMADD_SIZE_16BIT
	#define AT24c_Page_Size							128
	
#elif AT24Cxx == AT24C02
	#define AT24c_I2C_MEMADD_SIZE				I2C_MEMADD_SIZE_8BIT
	#define AT24c_Page_Size							128
	
#elif AT24Cxx == AT24C64
	#define AT24c_I2C_MEMADD_SIZE				I2C_MEMADD_SIZE_16BIT
	#define AT24c_Page_Size							32
#endif


#define AT24C_Delay(x)              HAL_Delay(x)



#include "main.h"


float at24_HAL_Read_Float(uint16_t DevAddress,uint16_t addr);
void at24_HAL_Write_Float(uint16_t DevAddress,uint16_t addr,float data);

HAL_StatusTypeDef at24_HAL_WriteBytes(uint16_t DevAddress,uint16_t MemAddress, uint8_t *pData,uint16_t TxBufferSize);
HAL_StatusTypeDef at24_HAL_ReadBytes(uint16_t DevAddress,uint16_t MemAddress, uint8_t *pData,uint16_t RxBufferSize);

//int at24_HAL_SequentialRead(I2C_HandleTypeDef *hi2c ,uint8_t DevAddress,uint16_t MemAddress,uint8_t *pData,uint16_t RxBufferSize);
HAL_StatusTypeDef at24_HAL_EraseMemFull(uint16_t DevAddress);
//int at24_HAL_WriteString(I2C_HandleTypeDef *hi2c,char *pString ,uint16_t MemAddress ,uint8_t length);
//int at24_HAL_ReadString(I2C_HandleTypeDef *hi2c,char *pString,uint16_t MemAddress,uint8_t length);


#endif /* DRIVERS_MYLIB_AT24_HAL_I2C_H_ */
