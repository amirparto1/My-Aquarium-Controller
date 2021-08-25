#include "delay_us.h"



extern TIM_HandleTypeDef us_timer_handle;


void delay_us(__IO uint32_t delay){
	__IO uint32_t tickstart = us_timer_handle.Instance->CNT;
	__IO uint32_t wait = delay;
  
  while((us_timer_handle.Instance->CNT - tickstart) < wait)
  {
  }
	
}


void delay_us_init(void){
	HAL_TIM_Base_Start(&us_timer_handle);
}

