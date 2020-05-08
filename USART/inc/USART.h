/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"


/** @addtogroup STM32F1
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define RX_BUF_SIZE 128

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile int8_t RX_END_LINE_FLAG = 0;
volatile uint8_t RXi;
volatile unsigned char RXc;
volatile unsigned char RX_BUF[RX_BUF_SIZE] = {'\0'};


/* Private function prototypes -----------------------------------------------*/
void USART1_GPIO_CONF(void);
void USART1_CONF(void);
void CLR_RX_BUFF(void);
void USART1_SEND(uint16_t data);
void USART1_SEND_STR(unsigned char *string);
void Configure_EXTI(void);