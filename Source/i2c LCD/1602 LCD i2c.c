#include "1602 LCD i2c.h"
#include <stdio.h>
#include <string.h>
extern I2C_HandleTypeDef lcd_i2c_handle;

uint8_t backlight_stat=led_on;


void lcd_dispfloat(float val,uint8_t size){
	size++;
	char str[size];
	snprintf(str,size,"%.2f",val);
	lcd_dispstr(str,size);
}



void lcd_dispval(int32_t val){
	uint8_t size=0;
	if(val<0 && val>-10) size=2;
	else if(val<-9 && val>-100) size=3;
	else if(val<-99 && val>-1000) size=4;
	else if(val<-999 && val>-10000) size=5;
	else if(val<-9999 && val>-100000) size=6;
	else if(val<-99999 && val>-1000000) size=7;
	else if(val<-999999 && val>-10000000) size=8;
	else if(val<-9999999 && val>-100000000) size=9;
	else if(val<-99999999 && val>-1000000000) size=10;
	else if(val<-999999999 && val>-2147483648) size=11; 
							
	
	else if(val<10 && val>-1) size=1;
	else if(val>9 && val<100) size=2;
	else if(val>99 && val<1000) size=3;
	else if(val>999 && val<10000) size=4;
	else if(val>9999 && val<100000) size=5;
	else if(val>99999 && val<1000000) size=6;
	else if(val>999999 && val<10000000) size=7;
	else if(val>9999999 && val<100000000) size=8;
	else if(val>99999999 && val<1000000000) size=9;
	else if(val>999999999 && val<2147483647) size=10;
	            
	char str[size];
	sprintf(str,"%d",val);
	lcd_dispstr(str,size);
}

void lcd_dispstr(char *str, uint8_t size){
	for(uint8_t i=0 ; i < size;i++)
  {
		lcd_send_data(str[i]);
  }
	
}

void lcd_init(void){
	HAL_Delay(110);
	lcd_send_cmd(0x30);
	HAL_Delay(6);
	lcd_send_cmd(0x30);
	lcd_send_cmd(0x30);
	lcd_send_cmd(0x20);
	lcd_send_cmd(0x20);
	lcd_send_cmd(0x08);
	lcd_send_cmd(0x01);
	HAL_Delay(3);
	lcd_send_cmd(0x06);
	lcd_send_cmd(0x0C);
	lcd_clear();
}

void lcd_shift(lcd_shift_enum shift , uint8_t n){
	for(uint8_t i=0;i<n;i++)
  {
		if (shift==shift_right) lcd_send_cmd(0x1E);
		if (shift==shift_left) lcd_send_cmd(0x18);
  }
}


void lcd_gotoxy(uint8_t x , uint8_t y){
	switch (y)
  {
  	case 0:
			lcd_send_cmd(0x80+0+x);
  		break;
  	case 1:
			lcd_send_cmd(0x80+40+x);
  		break;
  	default:
			lcd_send_cmd(0x80+40+x);
  		break;
  }
	
	
}

void lcd_gotoy(uint8_t y){
	switch (y)
  {
  	case 0:
			lcd_send_cmd(0x80+0);
  		break;
  	case 1:
			lcd_send_cmd(0x80+40);
  		break;
  	default:
			lcd_send_cmd(0x80+40);
  		break;
  }
	
}



void lcd_gotox(uint8_t x){
	if(x>15) x=15;
	lcd_send_cmd(0x80+x);
}




void lcd_backlight(lcd_backlight_enum led){

	backlight_stat=led;
	HAL_I2C_Master_Transmit (&lcd_i2c_handle, 0x4E, &backlight_stat, 1, 100);
	HAL_Delay(1);
}

void lcd_clear(void){
	lcd_send_cmd(0x01);
	HAL_Delay(1);
}

void lcd_home(void){
	lcd_send_cmd(0x02);
}

void lcd_cursor(cursor_status cursor){
	switch (cursor)
  {
  	case off:
			lcd_send_cmd(0x0c);
  		break;
  	case on:
			lcd_send_cmd(0x0e);
  		break;
		case blink:
			lcd_send_cmd(0x0d);
			break;
  	default:
  		break;
  }
}

void lcd_send_cmd (char cmd){
	char data_u, data_l;    
	uint8_t data_t[4];
	data_u = cmd&0xf0;    // select only upper nibble
	data_l = (cmd<<4)&0xf0;    // select only lower nibble
	data_t[0] = data_u|0x04|backlight_stat;  //en=1, rs=0, backlight=backlight_stat
	data_t[1] = data_u|backlight_stat;  //en=0, rs=0, backlight=backlight_stat
	data_t[2] = data_l|0x04|backlight_stat;  //en=1, rs=0, backlight=backlight_stat
	data_t[3] = data_l|backlight_stat;  //en=0, rs=0, backlight=backlight_stat
	HAL_I2C_Master_Transmit (&lcd_i2c_handle, 0x4E, (uint8_t *) data_t, 4, 100);
	HAL_Delay(1);
}
	 
void lcd_send_data (char data){
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = data&0xf0;    // upper data nibble
	data_l = (data<<4)&0xf0;    // lower data nibble
	data_t[0] = data_u|0x05|backlight_stat;  //en=1, rs=1, backlight=backlight_stat
	data_t[1] = data_u|0x01|backlight_stat;  //en=0, rs=1, backlight=backlight_stat
	data_t[2] = data_l|0x05|backlight_stat;  //en=1, rs=1, backlight=backlight_stat
	data_t[3] = data_l|0x01|backlight_stat;  //en=0, rs=1, backlight=backlight_stat
	HAL_I2C_Master_Transmit (&lcd_i2c_handle, 0x4E,(uint8_t *) data_t, 4, 100);
	HAL_Delay(1);
}

