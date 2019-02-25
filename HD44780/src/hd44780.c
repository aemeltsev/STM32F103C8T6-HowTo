#include "hd44780.h"

const unsigned char lcd44780_addLUT[4] = {0x80, 0xC0, 0x94, 0xD4};
unsigned char lcd44780_Address, lcd44780_Line;

void lcd44780_delay(unsigned int p)
{
	unsigned long i;
	for(i=0;i<(p*10);i++){}
}

void lcd44780_WriteNibble(unsigned char data)
{
	GPIOB->BSRR = ((data & 0x0F)<<HD44780_OFFSET);
	lcd44780_delay(200);
	lcd44780_EN_1;
	lcd44780_delay(100);
	lcd44780_EN_0;
	GPIOB->BRR = (0x0F<<HD44780_OFFSET);
}


void lcd44780_WriteByte(unsigned char data)
{
	lcd44780_WriteNibble(data >> 4);
	lcd44780_WriteNibble(data & 0x0F);
}

void lcd44780_GoToLine(char LineNum)
{
	lcd44780_RS_0;
	lcd44780_Address = lcd44780_addLUT[LineNum-1];
	lcd44780_WriteByte(lcd44780_Address);
	lcd44780_RS_1;
	lcd44780_Address = 0;
	lcd44780_Line = LineNum;
}

void lcd44780_ClearLCD(void)
{
	lcd44780_RS_0;
	lcd44780_WriteByte(0x01);
	lcd44780_delay(1000);
	lcd44780_RS_1;
	lcd44780_GoToLine(1);
}


void lcd44780_SetLCDPosition(char x, char y)
{
	lcd44780_RS_0;
	lcd44780_Address = lcd44780_addLUT[y] + x;
	lcd44780_WriteByte(lcd44780_Address);
	lcd44780_RS_1;
	lcd44780_Line = y+1;
}

void lcd44780_WriteCommand(unsigned char commandToWrite){
	
	lcd44780_RS_0;
	lcd44780_WriteByte(commandToWrite);
	lcd44780_delay(1000);
	lcd44780_RS_1;
	lcd44780_GoToLine(1);
	
}

void lcd44780_ShowChar(unsigned char c)
{
	lcd44780_RS_1;
	lcd44780_WriteByte(c);
	lcd44780_Address++;
	switch (lcd44780_Address)
	{
		case 20: lcd44780_GoToLine(2); break;
		case 40: lcd44780_GoToLine(3); break;
		case 60: lcd44780_GoToLine(4); break;
		case 80: lcd44780_GoToLine(1); break;
	}
}


void lcd44780_ShowStr(char *s)
{
	while (*s != 0) lcd44780_ShowChar(*s++);
}

void lcd44780_init(void)
{
	unsigned char i;
	
	HD44780_CLOCK_EN;
	
	//Configuring GPIOB.10
	GPIOB->CRH &= ~GPIO_CRH_MODE10;  //clear ranks MODE
	GPIOB->CRH &= ~GPIO_CRH_CNF10;   //clear ranks CNF
	GPIOB->CRH |=  GPIO_CRH_MODE10_1;//output, 2MHz
	GPIOB->CRH |=  GPIO_CRH_CNF10_0; //general purpose, open drain
	
	
	//Configuring GPIOB.11
	GPIOB->CRH &= ~GPIO_CRH_MODE11;  //clear ranks MODE
	GPIOB->CRH &= ~GPIO_CRH_CNF11;   //clear ranks CNF
	GPIOB->CRH |=  GPIO_CRH_MODE11_1;//output, 2MHz
	GPIOB->CRH |=  GPIO_CRH_CNF11_0; //general purpose, open drain
	
	//Configuring GPIOB.12
	GPIOB->CRH &= ~GPIO_CRH_MODE12;  //clear ranks MODE
	GPIOB->CRH &= ~GPIO_CRH_CNF12;   //clear ranks CNF
	GPIOB->CRH |=  GPIO_CRH_MODE12_1;//output, 2MHz
	GPIOB->CRH |=  GPIO_CRH_CNF12_0; //general purpose, open drain
	
	//Configuring GPIOB.13
	GPIOB->CRH &= ~GPIO_CRH_MODE13;  //clear ranks MODE
	GPIOB->CRH &= ~GPIO_CRH_CNF13;   //clear ranks CNF
	GPIOB->CRH |=  GPIO_CRH_MODE13_1;//output, 2MHz
	GPIOB->CRH |=  GPIO_CRH_CNF13_0; //general purpose, open drain
	
	//Configuring GPIOB.14
	GPIOB->CRH &= ~GPIO_CRH_MODE14;  //clear ranks MODE
	GPIOB->CRH &= ~GPIO_CRH_CNF14;   //clear ranks CNF
	GPIOB->CRH |=  GPIO_CRH_MODE14_1;//output, 2MHz
	GPIOB->CRH |=  GPIO_CRH_CNF14_0; //general purpose, open drain
	
	//Configuring GPIOB.15
	GPIOB->CRH &= ~GPIO_CRH_MODE15;  //clear ranks MODE
	GPIOB->CRH &= ~GPIO_CRH_CNF15;   //clear ranks CNF
	GPIOB->CRH |=  GPIO_CRH_MODE15_1;//output, 2MHz
	GPIOB->CRH |=  GPIO_CRH_CNF15_0; //general purpose, open drain
	
	lcd44780_EN_0;
	lcd44780_RS_0;
	lcd44780_delay(500);
	lcd44780_WriteNibble(0x33);
	lcd44780_WriteNibble(0x33);
	lcd44780_WriteNibble(0x33);
	lcd44780_WriteNibble(0x22);
	lcd44780_WriteByte(0x28);
	lcd44780_WriteByte(0x01);
	lcd44780_WriteByte(0x10);
	lcd44780_WriteByte(0x06);
	lcd44780_WriteByte(0x0C);
	for(i=0x40; i<0x5F; i++)
	{
		lcd44780_delay(5000);
		lcd44780_RS_0;
		lcd44780_WriteByte(i);
		lcd44780_delay(5000);
		lcd44780_ShowChar(0);
	}
	lcd44780_RS_1;
	lcd44780_ClearLCD();
}
