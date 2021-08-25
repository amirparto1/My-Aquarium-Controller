#include "Buzzer.h"



void Buzzer_Beep(enum beep beep, uint16_t delay){
	
	uint32_t tickstart = HAL_GetTick();
  uint32_t wait = delay;
  
  while((HAL_GetTick() - tickstart) < wait)
  {
		Buzzer_GPIO_Port->BSRR=Buzzer_Pin;
		delay_us(beep);
		Buzzer_GPIO_Port->BRR=Buzzer_Pin;
		delay_us(beep);
  }
	
}
