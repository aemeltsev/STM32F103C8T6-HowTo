#include "USARTPool.h"

//**************************************************************************************************
// Declarations and definitions
//**************************************************************************************************

void USART_Init(void)
{
	__enable_irq();
	NVIC_SetPriorityGrouping(0);
	
	#if (USART_INTRF_IN_USE == USART_INTRF1)
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN; //Enable the peripheral clock of GPIOA & alter function clock
	/*TX*/
	GPIOA->CRH		&= ~GPIO_CRH_CNF9;                         //Clear CNF bit 9
	GPIOA->CRH		|= GPIO_CRH_CNF9_1;                        //Set CNF bit 9 to [10] - alternate function output, push-pull
	GPIOA->CRH		|= GPIO_CRH_MODE9_0;                       //Set MODE bit 9 to [01] - 10MHz
	/*RX*/
	GPIOA->CRH		&= ~GPIO_CRH_CNF10;                        //Clear CNF bit 10
	GPIOA->CRH		|= GPIO_CRH_CNF10_0;                       //Set CNF bit 10 to [01] - general purpose output, open-drain
	GPIOA->CRH		&= ~GPIO_CRH_MODE10;                       //Set MODE bit 9 to Mode 01 = 10MHz
	
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;                    //Enable the peripheral clock USART1
  /*Configure USART1*/
  USART1->BRR = USART_BAUDRATE;                            //Baud rate generation - Tx/Rx baud = fck/(16*USARTDIV)
	USART1->CR1 &= ~USART_CR1_M;                             //8 data bits
	USART1->CR1 &= ~(USART_CR1_PS | USART_CR1_PCE);          //no parity 
	USART1->CR2 &= ~USART_CR2_STOP;                          //1 stop bit (STOP[13:12] = 00
	USART1->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);       //CTS and RTS hardware flow control disabled
  USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; //8 data bit, 1 start bit, 1 stop bit, no parity, reception mode
	
	NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority( 0, 1, 0 ));
	NVIC_EnableIRQ(USART1_IRQn);

	#endif // (USART_INTRF_IN_USE == USART_INTRF1)
	
	#if (USART_INTRF_IN_USE == USART_INTRF2)
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN; /*Enable the peripheral clock of GPIOA & alter function clock*/
  /*TX*/
	GPIOA->CRL    &= ~GPIO_CRL_CNF2;                   /*Clear CNF bit 2*/
	GPIOA->CRL    |= GPIO_CRL_CNF2_1;                  /*Set CNF bit 2 to [10] - alternate function output, push-pull*/
	GPIOA->CRL    |= GPIO_CRL_MODE2_0;                 /*Set MODE bit 2 to [01] - 10MHz*/
	/*RX*/
	GPIOA->CRL		&= ~GPIO_CRL_CNF3;                   /*Clear CNF bit 3*/
	GPIOA->CRL		|= GPIO_CRL_CNF3_0;                  /*Set CNF bit 3 to [01] - general purpose output, open-drain*/
	GPIOA->CRL		&= ~GPIO_CRL_MODE3;                  /*Set MODE bit 3 to Mode 01 = 10MHz*/
	
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;              /*Enable the peripheral clock USART2 */
	/*Configure USART2*/
  
	//uint16_t uartdiv = SystemCoreClock / 9600;
	USART2->BRR = 0x341;                               /*Baud rate generation - Tx/Rx baud = fck/(16*USARTDIV)*/
	USART2->CR1 &= ~USART_CR1_M;                       //8 data bits
	USART2->CR1 &= ~(USART_CR1_PS | USART_CR1_PCE);    //no parity
	USART2->CR2 &= ~USART_CR2_STOP;                    // 1 stop bit (STOP[13:12] = 00
	USART2->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE); //CTS and RTS hardware flow control disabled
	USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; /*8 data bit, 1 start bit, 1 stop bit, no parity, reception mode*/
	
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority( 0, 1, 0 ));
	NVIC_EnableIRQ(USART2_IRQn);
	
	#endif // (USART_INTRF_IN_USE == USART_INTRF2)
		
	#if (USART_INTRF_IN_USE == USART_INTRF3)
	RCC->APB1ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN; /*Enable the peripheral clock of GPIOA & alter function clock*/
	/*TX*/
	GPIOB->CRH &= ~GPIO_CRH_CNF10;               /*Clear CNF bits 10*/
	GPIOB->CRH |= GPIO_CRH_CNF10_1;              /*Set CNF bits 10 to [10] - alternate function output, push-pull*/
	GPIOB->CRH |= GPIO_CRH_MODE10_0;             /*Set MODE bits 10 to [01] - 10MHz*/
	/*RX*/
	GPIOB->CRH &= ~GPIO_CRH_CNF11;               /*Clear CNF bits 11*/
	GPIOB->CRH |= GPIO_CRH_CNF11_0;              /*Set CNF bits 11 to [01] - general purpose output, open-drain*/
	GPIOB->CRH &= ~GPIO_CRH_MODE11;              /*Set MODE bits 11 to Mode 01 = 10MHz*/
	
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;        /*Enable the peripheral clock USART3 */
	/*Configure USART3*/
  USART3->BRR = USART_BAUDRATE;                           /*Baud rate generation - Tx/Rx baud = fck/(16*USARTDIV)*/
	USART3->CR1 &= ~USART_CR1_M; //8 data bits
	USART3->CR1 &= ~(USART_CR1_PS | USART_CR1_PCE); //no parity 
	USART3->CR2 &= ~USART_CR2_STOP; // 1 stop bit (STOP[13:12] = 00
	USART3->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE); //CTS and RTS hardware flow control disabled
  USART3->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; /*8 data bit, 1 start bit, 1 stop bit, no parity, reception mode*/
	
	NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority( 0, 1, 0 ));
	NVIC_EnableIRQ(USART3_IRQn);
	
	#endif // (USART_INTRF_IN_USE == USART_INTRF3)
}

int _write(int handle, char* data, int size)
{
	int count = size;
	uint8_t i;
	for(i=0; i<count; i++)
	{
		#if (USART_INTRF_IN_USE == USART_INTRF1)
		while((USART1->SR &USART_SR_TXE)==USART_SR_TXE){};
		// Transmit data
		USART1->DR = (data[i] & (uint16_t)0x01FF);
		#endif
		
		#if (USART_INTRF_IN_USE == USART_INTRF2)
		while((USART2->SR &USART_SR_TXE)==USART_SR_TXE){};
		// Transmit data
		USART1->DR = (data[i] & (uint16_t)0x01FF);
		#endif
			
		#if (USART_INTRF_IN_USE == USART_INTRF3)
		while((USART1->SR &USART_SR_TXE)==USART_SR_TXE){};
		// Transmit data
		USART1->DR = (data[i] & (uint16_t)0x01FF);
		#endif
	}	
}

#if (USART_INTRF_IN_USE == USART_INTRF1)
extern void USART1_IRQHandler(void)
{
}
#endif // #if (USART_INTRF_NUMBER == USART_INTRF1)

#if (USART_INTRF_IN_USE == USART_INTRF2)
extern void USART2_IRQHandler(void)
{
	if( USART2->SR & USART_SR_RXNE ) 
	{
		char c = USART1->DR;
		rbuf_write(rb, c);
		if( c == '\r' ) { newline = 1; }
	}
}
#endif // #if (USART_INTRF_NUMBER == USART_INTRF2)

#if (USART_INTRF_IN_USE == USART_INTRF3)
extern void USART3_IRQHandler(void)
{
}
#endif // #if (USART_INTRF_NUMBER == USART_INTRF3)

