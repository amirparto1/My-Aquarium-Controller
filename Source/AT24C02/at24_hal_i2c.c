/* | -------------------------------- R2T Team Libraries -------------------------
 * | @Created On Sep,11,2015
 * | @File Name : at24_hal_i2c
 * | @Brief : STM32 HAL Driver for AT24 eeprom series
 * | @Copyright :
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * | @Author :  Amir Parto
 * | @Website : amirparto.blogfa.com
 * |
**/
/* Includes ------------------------------------------------------------------*/
//#include "stm32f1xx_hal.h"
#include "at24_hal_i2c.h"
#include <string.h>
#include <stdio.h>
#include "main.h"


#define delay     10
extern I2C_HandleTypeDef  AT24C_I2C_Handle;





/*void AT24C_Delay(uint16_t delay){
	for(uint32_t i=0;i<(60000*delay);i++){};
}*/


union myData
{
	float f;
	uint8_t  i[sizeof(float)];
};




float at24_HAL_Read_Float(uint16_t DevAddress,uint16_t addr)
{
	union myData q;
	at24_HAL_ReadBytes(DevAddress,addr,q.i,4);	
	return q.f;
}

void at24_HAL_Write_Float(uint16_t DevAddress,uint16_t addr,float data)
{
	union myData q;
	q.f=data;
	at24_HAL_WriteBytes(DevAddress,addr,q.i,4);
}






HAL_StatusTypeDef at24_HAL_WriteBytes(uint16_t DevAddress,uint16_t MemAddress, uint8_t *pData,uint16_t TxBufferSize)
{
	//uint8_t data[TxBufferSize];
	//strncpy((char*)data,(char*)pData,TxBufferSize);
	uint8_t firstcount=0;
	firstcount=(MemAddress/AT24c_Page_Size)+1;
	firstcount*=AT24c_Page_Size;
	firstcount=firstcount-MemAddress;	
	if(TxBufferSize>firstcount)
	{
		HAL_I2C_Mem_Write(&AT24C_I2C_Handle,(uint16_t)DevAddress,(uint16_t)MemAddress,AT24c_I2C_MEMADD_SIZE,pData,firstcount,1000);
		AT24C_Delay(delay);
		TxBufferSize-=firstcount;
		MemAddress+=firstcount;
		while(TxBufferSize>AT24c_Page_Size)
		{		
			HAL_I2C_Mem_Write(&AT24C_I2C_Handle,(uint16_t)DevAddress,(uint16_t)MemAddress,AT24c_I2C_MEMADD_SIZE,
												&pData[firstcount],AT24c_Page_Size,1000);
			AT24C_Delay(delay);
			MemAddress+=AT24c_Page_Size;
			firstcount+=AT24c_Page_Size;
			TxBufferSize-=AT24c_Page_Size;
		}
		if(TxBufferSize>0)
		{
			HAL_I2C_Mem_Write(&AT24C_I2C_Handle,(uint16_t)DevAddress,(uint16_t)MemAddress,AT24c_I2C_MEMADD_SIZE,
												&pData[firstcount],TxBufferSize,1000);
			AT24C_Delay(delay);
		}
	}
	else
	{
		HAL_I2C_Mem_Write(&AT24C_I2C_Handle,(uint16_t)DevAddress,(uint16_t)MemAddress,AT24c_I2C_MEMADD_SIZE,pData,TxBufferSize,1000);
		AT24C_Delay(delay);
	}
	return HAL_OK;
}




HAL_StatusTypeDef at24_HAL_ReadBytes(uint16_t DevAddress,uint16_t MemAddress, uint8_t *pData,uint16_t RxBufferSize)
{
	HAL_StatusTypeDef status=HAL_I2C_Mem_Read(&AT24C_I2C_Handle,DevAddress,MemAddress,AT24c_I2C_MEMADD_SIZE,pData,RxBufferSize,100);
	AT24C_Delay(delay);
	return status;
}



 /*
  * @brief               : This function handles Erase Full chip.
  * @param  hi2c         : Pointer to a I2C_HandleTypeDef structure that contains
  *                        the configuration information for the specified I2C.
  * @retval
  */

HAL_StatusTypeDef at24_HAL_EraseMemFull(uint16_t DevAddress)
{
	/*
	 * this may take will don't panic
	 */
	uint8_t EraseBuf[AT24c_Page_Size] ={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
													0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	int i;
	//uint8_t timeout=0;
	for (i =0 ; i<4096 ; i+=AT24c_Page_Size)
	{
		/*
		 * if you know,0xFF means that block is empty
		 */
		HAL_I2C_Mem_Write(&AT24C_I2C_Handle,DevAddress,(uint16_t )i,AT24c_I2C_MEMADD_SIZE,EraseBuf,AT24c_Page_Size,200);
		AT24C_Delay(delay);
	}
	return HAL_OK;
}
