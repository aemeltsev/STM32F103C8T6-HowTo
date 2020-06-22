/**
  ******************************************************************************
  * @file    HD44780.h 
  * @author  IRQ
  * @version V1.0
  * @date    05-October-2018
  * @brief   Provides a library to access a HD44780-based character LCD module.
  ******************************************************************************
  */
/**
	***********************************************************************************************************
	* HD44780 |  1. |  2. |  3. |  4. |  5. |  6. |  7. |  8. |  9. | 10. | 11. | 12. | 13. | 14. | 15. | 16. |
	          | VSS | VDD |  V0 |  RS | R/W |  E  | DB0 | DB1 | DB2 | DB3 | DB4 | DB5 | DB6 | DB7 |  A  |  K  |
						|  |     |     |     |     |     |     |     |     |     |     |     |     |     |     |     |  |
						|	 ^     ^     ^     ^     ^     ^     ^     ^     ^     ^     ^     ^     ^     ^     ^     ^  |
						|	GND   +5V   ADJ    |           |                             |     |     |     |    +5V   GND |
						|	 |                 |           |                             |     |     |     |           |  |
		STM32F1 | GND |           | PB11|  -  | PB10|  -  |  -  |  -  |  -  | PB12| PB13| PB14| PB15|  - 	|	GND	|
	***********************************************************************************************************
	*/
#include "stm32f10x.h" 

/* Define data and control lines  */
#define RS                             (1 << 11)
#define EN                             (1 << 10)
#define D4                             (1 << 12)
#define D5                             (1 << 13)
#define D6                             (1 << 14)
#define D7                             (1 << 15)

/* Enable Clock */
#define HD44780_CLOCK_EN               RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

/* COMMANDS */
#define HD44780_CLEAR	                 0x01    /*!< Clears display */
#define HD44780_HOME                   0x02    /*!< Sets DDRAM pointer to 0 */
#define HD44780_ENTRY_MODE			       0x04    /*!< Sets how the pointer is updated after a character write */
#define HD44780_CGRAM_SET				       0x40    /*!< Sets CGRAM address */
#define HD44780_DDRAM_SET				       0x80    /*!< Sets DDRAM address */
#define HD44780_OFFSET 			           0x12    /*!<*/

/* ENTRY_MODE Command parameters */
#define HD44780_EM_SHIFT_CURSOR        0x00    /*!< Shift cursor */
#define HD44780_EM_SHIFT_DISPLAY	     0x01    /*!< Shift display */
#define HD44780_EM_DECREMENT		       0x00    /*!< Increments pointer */
#define HD44780_EM_INCREMENT		       0x02    /*!< Decrements pointer */

/* DISPLAY Command parameters */
#define HD44780_DISPLAY_ONOFF		       0x08    
#define HD44780_DISPLAY_OFF		         0x00    /*!< Disables the display */
#define HD44780_DISPLAY_ON	           0x04    /*!< Enables the display */
#define HD44780_CURSOR_OFF		         0x00    /*!< Disables cursor */
#define HD44780_CURSOR_ON		           0x02    /*!< Enables cursor */
#define HD44780_CURSOR_NOBLINK	       0x00    /*!< Disables cursor blinking */
#define HD44780_CURSOR_BLINK	         0x01    /*!< Enables cursor blinking */

/* SHIFT Command parameters */
#define HD44780_DISPLAY_CURSOR_SHIFT   0x10    /*!< Cursor and display movement */
#define HD44780_SHIFT_CURSOR		       0x00    /*!< Shifts the display or shifts the cursor if not set */
#define HD44780_SHIFT_DISPLAY		       0x08    /*!< Shifts the display or shifts the cursor if not set */
#define HD44780_SHIFT_LEFT			       0x00    /*!< Shift to the left  */
#define HD44780_SHIFT_RIGHT		         0x04    /*!< Shift to the right */

/* FUNCTION Command parameters */
#define HD44780_FUNCTION_SET			     0x20    /*!< Screen type setup */
#define HD44780_FONT5x8				         0x00    /*!< 5x8 font */
#define HD44780_FONT5x10			         0x04    /*!< 5x10 font */
#define HD44780_ONE_LINE			         0x00    /*!< 1 line */
#define HD44780_TWO_LINE			         0x08    /*!< 2 lines */
#define HD44780_4_BIT				           0x00    /*!< 4 bit bus */
#define HD44780_8_BIT				           0x10    /*!< 8 bit bus */

/**
 ***************************************************************
 *
 * FUNCTIONS
 *
 ***************************************************************
 */

#define lcd44780_RS_1  GPIOB->BSRR = RS;       /*!< Set RS out */
#define lcd44780_EN_1  GPIOB->BSRR = EN;       /*!< Set EN out */
#define lcd44780_RS_0  GPIOB->BRR = RS;        /*!< Reset RS out */
#define lcd44780_EN_0  GPIOB->BRR = EN;        /*!< Reset EN out */

void lcd44780_delay(unsigned int p);
void lcd44780_delay_ns(uint32_t ns);
void lcd44780_ClearLCD(void);
void lcd44780_SetLCDPosition(char x, char y);
void lcd44780_ShowChar(unsigned char c);
void lcd44780_ShowStr(char *s);
void lcd44780_init(void);
void lcd44780_WriteNibble(unsigned char data);
void lcd44780_WriteByte(unsigned char data);
void lcd44780_GoToLine(char LineNum);
void lcd44780_WriteCommand(unsigned char commandToWrite);

