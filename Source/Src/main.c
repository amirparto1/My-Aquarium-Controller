/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ds18b20.h"
#include "Generaltec AC Remote Transmitter.h"
#include "1602 LCD i2c.h"
#include "delay_us.h"
#include "Buzzer.h"
#include "at24_hal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define light_pwm_handle    &htim3
#define light_pwm_channal   TIM_CHANNEL_1

#define set_temp_addr  10
//#define direction_addr  25;
//const uint8_t direction_value=33;
#define light_val_addr  50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */


double last_temprature=0 , set_temp=0;
uint16_t light_time=0;
uint16_t Pump_time=0;
uint16_t light_val=0;
uint8_t backlight_timer=0;
bool Pump_stat=true;
bool Pump_manual_stat=false;
bool light_stat=false;
bool enter_direction=false;
__IO uint8_t AC_stat=5;
extern uint8_t backlight_stat;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void temperature_init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	if(htim->Instance==TIM1){
//****************************bakclight code begin***************************
	if (backlight_stat==led_on){
		backlight_timer++;
		if(backlight_timer>=10){
			lcd_backlight(led_off);
		}
	}
//***************************backlight code end******************************


	
//***************************Light Code Begin********************************
		light_time++;
		if(light_time>32400 && !light_stat ){
			__HAL_TIM_SET_COMPARE(light_pwm_handle,light_pwm_channal,light_val);

			Buzzer_Beep(beep4,50);
			light_time=0;
			light_stat=true;
		}
		else if(light_time>10800 && light_stat)
		{
			Buzzer_Beep(beep5,50);
			__HAL_TIM_SET_COMPARE(light_pwm_handle,light_pwm_channal,1);
			light_time=0;
			light_stat=false;	
		}
		//***************************Light Code End********************************
		
		
		//***************************Pump Code Begin********************************
		Pump_time++;
//		if(!Pump_manual_stat){
//			if(Pump_time>3600 && Pump_stat ){
//				BTN_Pump_GPIO_Port->BRR = (uint32_t)Pump_Pin;
//				Pump_time=0;
//				Pump_stat=false;
//			}
//			else if (Pump_time>300 && !Pump_stat){
//				BTN_Pump_GPIO_Port->BSRR = (uint32_t)Pump_Pin;
//				Pump_time=1;
//				Pump_stat=true;
//			}		
//		}
	 if(Pump_manual_stat && Pump_time>500){
			BTN_Pump_GPIO_Port->BSRR = (uint32_t)Pump_Pin;
			Pump_time=0;
			Pump_stat=true;
			Pump_manual_stat=false;
		}
		//***************************Light Code End********************************
		
		
		if(Ds18b20_ManualConvert())
		{
			if(last_temprature!=ds18b20[0].Temperature)
			{
				last_temprature=ds18b20[0].Temperature;
				if(((last_temprature>=(set_temp+0.35)) || (last_temprature<(set_temp-0.1))) && 
																		(last_temprature<60) && (last_temprature>5)){
					AC_stat=5;
				}
				if((last_temprature>=(set_temp+.5)) && (last_temprature<60) && AC_stat!=1){
					NEC_AC_ON();
					AC_stat=1;
				}
				if((last_temprature<=(set_temp)) && (last_temprature>5) && AC_stat!=0){
					NEC_AC_OFF();
					AC_stat=0;
				}
				
				if((last_temprature<(set_temp-0.1)) && (last_temprature>5)){
					Heater_GPIO_Port->BSRR = (uint32_t)Heater_Pin;
				}
				if((last_temprature>=(set_temp)) && (last_temprature>5)){
					Heater_GPIO_Port->BRR = (uint32_t)Heater_Pin;	
				}
				lcd_home();
				lcd_dispstr("RealT=",6);
				lcd_dispfloat(last_temprature,5);
				lcd_gotoxy(0,1);
				lcd_dispstr("SetT=",5);
				lcd_dispfloat(set_temp,5);
				
			}
		}
	}
}







void button_release_wait(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	HAL_Delay(100);
	while((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET);
}




void light_eeprom_write(uint16_t light){
	uint8_t data[2];
	data[0]=light >> 8;
	data[1]=light;
	at24_HAL_WriteBytes(AT24C_ADDRESS,light_val_addr,data,2);
}




GPIO_PinState push_button_read(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	if((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET){
		HAL_Delay(30);
		if((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET){
			lcd_backlight(led_on);
			backlight_timer=0;
			return GPIO_PIN_SET;
		}
	}
	return GPIO_PIN_RESET;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	HAL_Delay(100);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	delay_us_init();
	lcd_init();
	lcd_dispstr("Starting",8);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	
	for(uint8_t i=0;i<7;i++){
		HAL_IWDG_Refresh(&hiwdg);
		if(!Ds18b20_Init()){
			if(push_button_read(BTN_Enter_GPIO_Port,BTN_Enter_Pin))	break;
			Buzzer_Beep(beep2,40);
			if(i>4){
				lcd_clear();
				lcd_dispstr("Failed",6);
				while(1)Buzzer_Beep(beep2,40);
			}
		}
		else{
			Ds18b20_ManualConvert();
			HAL_IWDG_Refresh(&hiwdg);
			break;
		}
	}
	temperature_init();
	
	lcd_clear();
	lcd_gotoxy(15,1);
	lcd_send_data('T');
	last_temprature=12;
	
	HAL_TIM_PeriodElapsedCallback(&htim1);
	HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */
	
	
	
	
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_IWDG_Refresh(&hiwdg);
		
		if(push_button_read(BTN_Up_GPIO_Port,BTN_Up_Pin)){
				if(!enter_direction){
				
					if(set_temp<33.9)set_temp+=0.1;
					else (set_temp=34);
					last_temprature++;
					AC_stat=5;
					at24_HAL_Write_Float(AT24C_ADDRESS,set_temp_addr,set_temp);
					//lcd_gotox(14);
					//lcd_send_data('U');
					lcd_gotoy(1);
					lcd_dispstr("SetT=",5);
					lcd_dispfloat(set_temp,5);
				}
				else{
					if(light_val<1260){ 
						if(light_val<1200)light_val+=50;
						else light_val+=10;
					}
					else light_val=1271;
					light_eeprom_write(light_val);
					if(light_stat)__HAL_TIM_SET_COMPARE(light_pwm_handle,light_pwm_channal,light_val);
				}
				
				//button_release_wait(BTN_Up_GPIO_Port,BTN_Up_Pin);
				HAL_Delay(150);
			}
		
		
		
		
//*************************************************************************
			if(push_button_read(BTN_Down_GPIO_Port,BTN_Down_Pin)){
				if(!enter_direction){
					if(set_temp>20.1)set_temp-=0.1;
					else set_temp=20;
					last_temprature++;
					AC_stat=5;
					at24_HAL_Write_Float(AT24C_ADDRESS,set_temp_addr,set_temp);
					//lcd_gotox(14);
					//lcd_send_data('D');
					lcd_gotoy(1);
					lcd_dispstr("SetT=",5);
					lcd_dispfloat(set_temp,5);
				}
				else{
					if(light_val>50){
						if(light_val<1220)light_val-=50;
						else light_val-=10; 
					}
					else light_val=1;
					light_eeprom_write(light_val);
					if(light_stat)__HAL_TIM_SET_COMPARE(light_pwm_handle,light_pwm_channal,light_val);
				}
				//button_release_wait(BTN_Down_GPIO_Port,BTN_Down_Pin);
				HAL_Delay(150);
		}
		
		//*************************************************************************
		if(push_button_read(BTN_Light_GPIO_Port,BTN_Light_Pin)){
				if(!light_stat){
					Buzzer_Beep(beep4,50);
					__HAL_TIM_SET_COMPARE(light_pwm_handle,light_pwm_channal,light_val);
					light_time=0;
					light_stat=true;
				}
				else if(light_stat)
				{
					Buzzer_Beep(beep5,50);
					__HAL_TIM_SET_COMPARE(light_pwm_handle,light_pwm_channal,1);
					light_time=0;
					light_stat=false;	
				}
				button_release_wait(BTN_Light_GPIO_Port,BTN_Light_Pin);
			
		}
		//*************************************************************************
		
		if(push_button_read(BTN_Pump_GPIO_Port,BTN_Pump_Pin)){
				if(!Pump_stat){
					Buzzer_Beep(beep4,50);
					HAL_GPIO_WritePin(BTN_Pump_GPIO_Port,Pump_Pin,GPIO_PIN_SET);
					Pump_manual_stat=false;
					Pump_time=0;
					Pump_stat=true;
				}
				else if(Pump_stat)
				{
					Buzzer_Beep(beep5,50);
					HAL_GPIO_WritePin(BTN_Pump_GPIO_Port,Pump_Pin,GPIO_PIN_RESET);
					Pump_time=0;
					Pump_stat=false;	
					Pump_manual_stat=true;
				}
				button_release_wait(BTN_Pump_GPIO_Port, BTN_Pump_Pin);
		}
		
		
	//*************************************************************************	
		
		if(push_button_read(BTN_Enter_GPIO_Port,BTN_Enter_Pin)){
				enter_direction=!enter_direction;
				if(enter_direction){
					lcd_gotoxy(15,1);
					lcd_send_data('L');
				}
				else{
					lcd_gotoxy(15,1);
					lcd_send_data('T');
				}
				HAL_Delay(200);
				button_release_wait(BTN_Enter_GPIO_Port, BTN_Enter_Pin);
		}
		
		
					
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
   // Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  //RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  //RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
  //  Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
  //  Error_Handler();
  }
  /** Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303EFD;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
  //  Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
   // Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
  //  Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 3500;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
   // Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1499;
  //htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 63999;
  //htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim1.Init.RepetitionCounter = 0;
  //htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    //Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
   // Error_Handler();
  }
  //sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  //sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    //Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  //htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1271;
  //htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
   // Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
   // Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
   // Error_Handler();
  }
  //sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    //Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
   // Error_Handler();
  }
  sConfigOC.Pulse = 480;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
   // Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 48-1;
  //htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 0xffff;
  //htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
   // Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Heater_Pin|Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Pump_GPIO_Port, Pump_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Heater_Pin Pump_Pin */
  GPIO_InitStruct.Pin = Heater_Pin|Pump_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_Pump_Pin BTN_Enter_Pin BTN_Down_Pin BTN_Up_Pin */
  GPIO_InitStruct.Pin = BTN_Pump_Pin|BTN_Enter_Pin|BTN_Down_Pin|BTN_Up_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  //GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Light_Pin */
  GPIO_InitStruct.Pin = BTN_Light_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  //GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_Light_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  //GPIO_InitStruct.Pin = DS18B20_Pin;
  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void temperature_init(void){
	/*uint8_t mem_read=5;
	at24_HAL_ReadBytes(&hi2c1,AT24C_ADDRESS,direction_addr,&mem_read,1); 
	if(mem_read!=direction_value){
		set_temp=26.5;
		at24_HAL_EraseMemFull(&hi2c1);
		uint8_t i=direction_value;
		at24_HAL_WriteBytes(&hi2c1,AT24C_ADDRESS,direction_addr,&i,1);
		at24_HAL_Write_double(&hi2c1,AT24C_ADDRESS,set_temp_addr,set_temp);
		mem_read=0;
		if(at24_HAL_ReadBytes(&hi2c1,AT24C_ADDRESS,direction_addr,&mem_read,1)== HAL_OK){
			//lcd_dispstr("IF",2);
			//lcd_dispval(mem_read);
		}
	}
	else{*/
		set_temp=at24_HAL_Read_Float(AT24C_ADDRESS,set_temp_addr);
		HAL_IWDG_Refresh(&hiwdg);
		uint8_t light[2]={0,0};
		at24_HAL_ReadBytes(AT24C_ADDRESS,light_val_addr,light,2);
		light_val=light[0]<<8;
		light_val|=light[1];
		//lcd_gotoy(1);
		//lcd_dispstr("else",4);
		//lcd_dispdouble(set_temp,7);		
	//}
	//at24_HAL_EraseMemFull(&hi2c1);
	
	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
//void Error_Handler(void)
//{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
//}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
