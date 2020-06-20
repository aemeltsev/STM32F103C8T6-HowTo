/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"


/** @addtogroup STM32F1
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void USART1GpioConf(void);
void USART1Conf(void);
void USARTSend(uint16_t data);
void USARTSendString(char *string);
uint16_t USARTGet(void);