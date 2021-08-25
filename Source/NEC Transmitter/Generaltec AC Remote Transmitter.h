#include "main.h"          // change to whatever MCU you use
#include "delay_us.h"
//#include "tim.h"

#define NEC_Timer_Handle   htim3
#define NEC_PWM_Channal  TIM_CHANNEL_2


void NEC_transmit(uint8_t *data);
void NEC_AC_ON(void);
void NEC_AC_OFF(void);


