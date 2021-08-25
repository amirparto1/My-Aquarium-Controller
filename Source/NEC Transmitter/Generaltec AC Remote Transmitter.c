
#include "Generaltec AC Remote Transmitter.h"




uint8_t code_on[] ={0xC3,0x11,0x00,0x00,0x05,0x00,0x02,0x00,0x00,0x04,0x00,0xA0,0x0A}; //dry 25 HorOn VerOn speedAuto
uint8_t code_off[]={0xC3,0x11,0x00,0x00,0x05,0x00,0x02,0x00,0x00,0x00,0x00,0xA0,0x0C}; //off



extern TIM_HandleTypeDef NEC_Timer_Handle;


void NEC_AC_ON()
{
		NEC_transmit(code_on);
}

void NEC_AC_OFF()
{
		NEC_transmit(code_off);
}


void NEC_transmit(uint8_t *data)
{
	uint32_t DC=NEC_Timer_Handle.Init.Period * 0.39;
	__HAL_TIM_SET_COMPARE(&NEC_Timer_Handle,NEC_PWM_Channal,DC);
	uint8_t j=0,i=0,bit_check=0;
	HAL_TIM_PWM_Start(&NEC_Timer_Handle,NEC_PWM_Channal);
	delay_us(9000);
	HAL_TIM_PWM_Stop(&NEC_Timer_Handle,NEC_PWM_Channal);
	delay_us(4500);
	
	for(j=0 ; j<13 ; j++)
	{
		for(i=0 ; i<8 ; i++)
		{
			bit_check = data[j] << i;
			if ((bit_check & 0x80)!=0){
				HAL_TIM_PWM_Start(&NEC_Timer_Handle,NEC_PWM_Channal);
				delay_us(600);
				HAL_TIM_PWM_Stop(&NEC_Timer_Handle,NEC_PWM_Channal);
				delay_us(1600);
			}
			else{
				HAL_TIM_PWM_Start(&NEC_Timer_Handle,NEC_PWM_Channal);
				delay_us(600);
				HAL_TIM_PWM_Stop(&NEC_Timer_Handle,NEC_PWM_Channal);
				delay_us(500);
			}					
		}
	}
	HAL_TIM_PWM_Start(&NEC_Timer_Handle,NEC_PWM_Channal);
	delay_us(600);
	HAL_TIM_PWM_Stop(&NEC_Timer_Handle,NEC_PWM_Channal);
	delay_us(500);	
}



