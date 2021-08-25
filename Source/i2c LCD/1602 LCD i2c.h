#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define lcd_i2c_handle  hi2c1

 
enum cursor_status{
	off,
	on,
	blink	
};

enum lcd_backlight{
	led_off=0x00,
	led_on=0x08
};

enum lcd_shift{
	shift_left,
	shift_right
};

void lcd_dispfloat(float val,uint8_t size);
void lcd_dispval(int32_t val);
void lcd_dispstr(char *str, uint8_t size);
void lcd_init(void);
void lcd_shift(enum lcd_shift shift , uint8_t n);
void lcd_gotoxy(uint8_t x , uint8_t y);
void lcd_gotoy(uint8_t y);
void lcd_gotox(uint8_t x);
void lcd_send_cmd (char cmd);
void lcd_send_data (char data);
void lcd_clear(void);
void lcd_home(void);
void lcd_cursor(enum cursor_status cursor);
void lcd_backlight(enum lcd_backlight led);
