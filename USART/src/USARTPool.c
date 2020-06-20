#include "USARTPool.h"

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the USART1 pins on GPIO PA9 PA10
  * @param  None
  * @return None
  */
void USART1GpioConf(void)
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
  * @return None
  */
void USART1Conf(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;          /*Enable the peripheral clock USART1 */
  /*Configure USART1*/
  USART1->BRR = 0x341;                           /*Baud rate generation - Tx/Rx baud = fck/(16*USARTDIV)*/
	USART1->CR1 &= ~USART_CR1_M; //8 data bits
	USART1->CR1 &= ~(USART_CR1_PS | USART_CR1_PCE); //no parity 
	USART1->CR2 &= ~USART_CR2_STOP; // 1 stop bit (STOP[13:12] = 00
	USART1->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE); //CTS and RTS hardware flow control disabled
  USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; /*8 data bit, 1 start bit, 1 stop bit, no parity, reception mode*/

}
/**
  * @brief  This function :
             - Transmit data to usart
  * @param  None
  * @return None
  */
void USARTSend(uint16_t data)
{
	while((USART1->SR & USART_SR_TXE)==USART_SR_TXE){};
	/* Transmit Data */
	USART1->DR = (data & (uint16_t)0x01FF);
}
/**
  * @brief  This function :
             - Transmit string to usart, in while cycle
  * @param  char string
  * @return None
  */
void USARTSendString(char *string)
{
	uint8_t i=0;
	/* Transmit String */
	while(string[i])
	{
		USARTSend(string[i]);
		i++;
	}
}
/**
  * @brief  This function :
             - Get data from usart
  * @param  None
  * @return uint16_t data
  */
uint16_t USARTGet(void)
{
	while((USART1->SR & USART_SR_RXNE)==USART_SR_RXNE){};
	/* Receive Data */
  return (uint16_t)(USART1->DR & (uint16_t)0x01FF);

}

