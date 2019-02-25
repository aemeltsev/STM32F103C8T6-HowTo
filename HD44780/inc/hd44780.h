#include "stm32f10x.h" 

#define RS                             (1 << 11)
#define EN                             (1 << 10)
#define D4                             (1 << 12)
#define D5                             (1 << 13)
#define D6                             (1 << 14)
#define D7                             (1 << 15)

#define HD44780_CLOCK_EN               RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

#define HD44780_CLEAR	                 0x01
#define HD44780_HOME                   0x02

#define HD44780_ENTRY_MODE			       0x04
    #define HD44780_EM_SHIFT_CURSOR       0
    #define HD44780_EM_SHIFT_DISPLAY	    1
    #define HD44780_EM_DECREMENT		      0
    #define HD44780_EM_INCREMENT		      2
		
#define HD44780_DISPLAY_ONOFF		       0x08
    #define HD44780_DISPLAY_OFF		        0
    #define HD44780_DISPLAY_ON	          4
    #define HD44780_CURSOR_OFF		        0
    #define HD44780_CURSOR_ON		          2
    #define HD44780_CURSOR_NOBLINK	      0
    #define HD44780_CURSOR_BLINK	        1

#define HD44780_DISPLAY_CURSOR_SHIFT   0x10
    #define HD44780_SHIFT_CURSOR		      0
    #define HD44780_SHIFT_DISPLAY		      8
    #define HD44780_SHIFT_LEFT			      0
    #define HD44780_SHIFT_RIGHT		        4
		
#define HD44780_FUNCTION_SET			     0x20
    #define HD44780_FONT5x7				        0
    #define HD44780_FONT5x10			        4
    #define HD44780_ONE_LINE			        0
    #define HD44780_TWO_LINE			        8
    #define HD44780_4_BIT				          0
    #define HD44780_8_BIT				          16
		
#define HD44780_CGRAM_SET				       0x40
#define HD44780_DDRAM_SET				       0x80

#define HD44780_OFFSET 			           12

#define lcd44780_RS_1  GPIOB->BSRR = RS;
#define lcd44780_EN_1  GPIOB->BSRR = EN;
#define lcd44780_RS_0  GPIOB->BRR = RS;
#define lcd44780_EN_0  GPIOB->BRR = EN;


void lcd44780_delay(unsigned int p);
void lcd44780_ClearLCD(void);
void lcd44780_SetLCDPosition(char x, char y);
void lcd44780_ShowChar(unsigned char c);
void lcd44780_ShowStr(char *s);
void lcd44780_init(void);
void lcd44780_WriteNibble(unsigned char data);
void lcd44780_WriteByte(unsigned char data);
void lcd44780_GoToLine(char LineNum);
void lcd44780_WriteCommand(unsigned char commandToWrite);

