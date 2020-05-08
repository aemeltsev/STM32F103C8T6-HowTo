#include "USART.h"


/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the USART1 pins on GPIO PA9 PA10
  * @param  None
  * @retval None
  */
void USART1_GPIO_CONF(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN; /*Enable the peripheral clock of GPIOA & alter function clock*/
	
	/*TX*/
	GPIOA->CRH		&= ~GPIO_CRH_CNF9;               /*Clear CNF bit 9*/
	GPIOA->CRH		|= GPIO_CRH_CNF9_1;              /*Set CNF bit 9 to [10] - alternate function output, push-pull*/
	GPIOA->CRH		|= GPIO_CRH_MODE9_0;             /*Set MODE bit 9 to [01] - 10MHz*/
	/*RX*/
	GPIOA->CRH		&= ~GPIO_CRH_CNF10;              /*Clear CNF bit 10*/
	GPIOA->CRH		|= GPIO_CRH_CNF10_0;             /*Set CNF bit 10 to [01] - general purpose output, open-drain*/
	GPIOA->CRH		&= ~GPIO_CRH_MODE10;             /*Set MODE bit 9 to Mode 01 = 10MHz*/
}
/**
  * @brief  This function configures USART1.
  * @param  None
  * @retval None
  */
void USART1_CONF(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;          /*Enable the peripheral clock USART1 */

  /*Configure USART1*/
  USART1->BRR = 0x341;                           /*Baud rate generation - Tx/Rx baud = fck/(16*USARTDIV)*/
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; /*8 data bit, 1 start bit, 1 stop bit, no parity, reception mode*/
  /*Configure IT*/
  NVIC_SetPriority(USART1_IRQn, 0);              /*Set priority for USART1_IRQn, priority 0*/
  NVIC_EnableIRQ(USART1_IRQn);                   /*Enable USART1_IRQn*/
}

/**
  * @brief  This function :
             - Clear receiver buffer
  * @param  None
  * @retval None
  */
void CLR_RX_BUFF(void)
{
	for(RXi=0; RXi<RX_BUF_SIZE; RXi++)
	{ 
		RX_BUF[RXi] = '\0';
	}
	RXi = 0;
}
/**
  * @brief  This function :
             - Receive byte in usart
  * @param  None
  * @retval None
  */
void USART1_SEND(uint16_t data)
{
	while(!(USART1->SR & USART_SR_TC)){};
	
	/* Transmit Data */
	USART1->DR = (data & (uint16_t)0x01FF);
}
/**
  * @brief  This function :
             - Receive string to usart, in while cycle
  * @param  char string
  * @retval None
  */
void USART1_SEND_STR(unsigned char *string)
{
	uint8_t i=0;
	/* Transmit String */
	while(string[i])
	{
		USART1_SEND(string[i]);
		i++;
	}
}
/**
  * @brief  This function :
             - Receive string to usart, in while cycle
  * @param  char string
  * @retval None
  */
uint16_t USART_GET(void)
{
	/* Receive Data */
  return (uint16_t)(USART1->DR & (uint16_t)0x01FF);
}

/**
  * @brief  This function :
             - Receive string to usart, in while cycle
  * @param  char string
  * @retval None
  */
void USARTGetString(USART_TypeDef* USARTx)
{
	if((USARTx->SR & USART_SR_RXNE) != (uint16_t)RESET)
	{
		RXc = (uint16_t)(USARTx->DR & (uint16_t)0x01FF);
		RX_BUF[RXi] = RXc;
		RXi++;
		
		if(RXc !=13)
		{
			if(RXi > RX_BUF_SIZE-1)
			{
				CLR_RX_BUFF();
			}
		}
		else
		{
			RX_END_LINE_FLAG = 0x01;
		}
	}
}
/**
  * @brief  This function :
             - Receive string to usart, in while cycle
  * @param  char string
  * @retval None
  */
__inline long USARTGetInt(USART_TypeDef* USARTx)
{
	unsigned char temp = 0x00, index = 0x00, flag = 0x00;
	long value = 0x00;
	temp = USARTGet(USARTx);
	if(temp=='-')
	{
		flag = 1;
		temp = USARTGet(USARTx);
		index++;
	}
	do
	{
		index++;
		if((47<temp)&&(temp<58))
		{
			value = value*10+temp-48;
		}
		else
		{
			index = 255;
		}
		if(index<7)
		{
			temp = USARTGet(USARTx);
		}
	}while(index<7);
	/* while(index<7)
	{
		value = value*10+temp;
		if(flag==0)
		{
			flag = 2;
		}
		else
		{
			temp = USARTGet(USARTx);
		}
		index++;
		if((47<temp)&&(temp<58))
		{
			temp = temp - 48;
		}
		else
		{
			index = 255;
		}
	} */ 
	if(flag==1)
	{
		value =-value;
	}
	return value;
}
/**
  * @brief  This function :
             - Output the number into USART
             - Maximum the number length to 6 symbol
  * @param  char string
  * @retval None
  */
__inline void USARTSendInt(USART_TypeDef* USARTx,long data)
{
	unsigned long digit = 0x989680;
	char temp, flag = 0x00;
	if(data<0)
	{
		USARTSend(USARTx,'-');
		data =-data;
	}
	do
	{
		data = data%digit;
		digit = digit/10;
		temp = data/digit;
		if(temp!=0x00)
		{
			flag  = 0x01;
		}
		if(flag==1)
		{
			USARTSend(USARTx,temp+'0');
		}
	}
	while(digit>0x01);
}

void USART1_IRQHandler(void)
{
	if (USART1->SR & USART_SR_RXNE)
	{
	USART1->DR = (USART1->DR)+1;
	}
}

