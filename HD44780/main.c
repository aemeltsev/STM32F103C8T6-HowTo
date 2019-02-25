#include "stm32f10x.h" 
#include "stdio.h"
#include "stdint.h"
#include "hd44780.h"

int main(void){
	
	lcd44780_init();
	lcd44780_SetLCDPosition(0,1);
	lcd44780_ShowStr("Hi! I'am stm32");
	lcd44780_delay(10000000);
	lcd44780_WriteCommand(HD44780_CLEAR);
	lcd44780_ShowStr("I use command");
	while(1){
		
	}
}