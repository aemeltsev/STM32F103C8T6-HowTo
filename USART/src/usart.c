#include "usart.h"

//**************************************************************************************************
// Declarations and definitions
//**************************************************************************************************

void usart_init(void)
{	
	#if (USART_INTRF_IN_USE == USART_INTRF1)
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                   //Enable the peripheral clock of GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;                 //Enable the peripheral clock USART1
	
	/*PA9 - set TX1*/
	GPIOA->CRH    &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);   // reset PA9 - TX
	GPIOA->CRH    &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10); // reset PA10 - RX
	
	GPIOA->CRH		|= GPIO_CRH_MODE9_1;                    //PA9: MODE: 0b10 output up to 2Mhz
	GPIOA->CRH		|= GPIO_CRH_CNF9_1;                     //PA9: CNF: 0b10 alt-function push-pull
	
	/*PA10 - set RX1*/
	GPIOA->CRH		|= GPIO_CRH_CNF10_1;                    //PA10: MODE: 0b00 input - CNF: 0b10 with resistor pull-up
	GPIOA->ODR		|= GPIO_ODR_ODR10;                      //Set MODE bit 9 to Mode 01 = 10MHz
	
  /*Configure USART1*/
	/*Baud rate generation - Tx/Rx baud = fck/(16*USARTDIV)
	  For example, we have a bus frequency of 36 MHz, and we should have a boudrate of 19200 baud.
		
		(36000000/19200)/16 = 117.18
		
		The integer part will be 117, but the fractional part will need to be restored by multiplying by 16
		and rounding to the nearest integer:
		
		0.18 * 16 = 2.88 -> 3
		
		We translate into hex and get 0x75 and 0x3 i.e. USART_BRR = 0x00000753
	*/
	USART2->BRR = 0xEA0;
	USART1->CR1 &= ~USART_CR1_M;                             //8 data bits
	USART1->CR1 &= ~(USART_CR1_PS | USART_CR1_PCE);          //no parity 
	USART1->CR2 &= ~USART_CR2_STOP;                          //1 stop bit (STOP[13:12] = 00
	USART1->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);       //CTS and RTS hardware flow control disabled
  USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; //8 data bit, 1 start bit, 1 stop bit, no parity, reception mode
	
	NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority( 0, 1, 0 ));
	NVIC_EnableIRQ(USART1_IRQn);

	#endif // (USART_INTRF_IN_USE == USART_INTRF1)
	
	#if (USART_INTRF_IN_USE == USART_INTRF2)
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;                  // Enable the peripheral clock of GPIOA & alter function clock
	/* Enable the peripheral clock USART2 */
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  
	
  /* PA2 - set TX2 */
	/* PA2: MODE: 0b01 10Mhz output */
	/* Clear CNF bit 2 */
	/* PA2: CNF: 0b10 - Alt-function output, Push-pull */
	GPIOA->CRL    |= GPIO_CRL_MODE2_0;
	GPIOA->CRL    &= ~GPIO_CRL_CNF2;
	GPIOA->CRL    |= GPIO_CRL_CNF2_1;

	/* PA3 - set RX2*/
	/* Clear CNF bit 3 */
	/* PA3: CNF: 0b01 floating input */
	/* PA3: MODE: 0b00 input mode */
	GPIOA->CRL    &= ~(GPIO_CRL_CNF3);
	GPIOA->CRL		|= GPIO_CRL_CNF3_0;
	GPIOA->CRL		&= ~(GPIO_CRL_MODE3);
	
	/*Don't remap the pins*/
	AFIO->MAPR &= ~AFIO_MAPR_USART2_REMAP;

	/*Configure USART2*/
	/*Baud rate generation - Tx/Rx baud = fck/(16*USARTDIV)
	  For example, we have a bus frequency of 36 MHz, and we should have a boudrate of 19200 baud.
		
		(36000000/19200)/16 = 117.18
		
		The integer part will be 117, but the fractional part will need to be restored by multiplying by 16
		and rounding to the nearest integer:
		
		0.18 * 16 = 2.88 -> 3
		
		We translate into hex and get 0x75 and 0x3 i.e. USART_BRR = 0x00000753
	*/
	uart_set_baudrate(USART2, PERPH_CLK, USART_BAUDRATE);
	USART2->CR1 &= ~USART_CR1_M;                       // 8 data bits
	USART2->CR1 &= ~(USART_CR1_PS | USART_CR1_PCE);    // no parity
	USART2->CR2 &= ~USART_CR2_STOP;                    // 1 stop bit (STOP[13:12] = 00
	USART2->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE); // CTS and RTS hardware flow control disabled								
	USART2->CR1 |= USART_CR1_RE | USART_CR1_TE |
                 USART_CR1_RXNEIE | USART_CR1_UE;
									 
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority( 0, 1, 0 ));
	NVIC_EnableIRQ(USART2_IRQn);
	
	#endif // (USART_INTRF_IN_USE == USART_INTRF2)
		
	#if (USART_INTRF_IN_USE == USART_INTRF3)
	RCC->APB1ENR |= RCC_APB2ENR_IOPBEN;                   // Enable the peripheral clock of GPIOB
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;                 // Enable the peripheral clock USART3
	
	GPIOB->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);    // reset PB10
	GPIOB->CRH &= ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11);    // reset PB11
	
	/*PB10 - set TX3*/
	GPIOB->CRH    |= GPIO_CRH_MODE10_1;                   // PB10: MODE: 0b10 output up to 2Mhz
	GPIOB->CRH    |= GPIO_CRH_CNF10_1;                    // PB10: CNF: 0b10 alt-function push-pull
	
	/*PB11 - set RX3*/
	GPIOB->CRH    |= GPIO_CRH_CNF11_1;                    // PB11: MODE: 0b00 input - CNF: 0b10 with resistor pull-up
	GPIOB->ODR    |= GPIO_ODR_ODR11;
	
	/*Configure USART3*/
	/*Baud rate generation - Tx/Rx baud = fck/(16*USARTDIV)
	  For example, we have a bus frequency of 36 MHz, and we should have a boudrate of 19200 baud.
		
		(36000000/19200)/16 = 117.18
		
		The integer part will be 117, but the fractional part will need to be restored by multiplying by 16
		and rounding to the nearest integer:
		
		0.18 * 16 = 2.88 -> 3
		
		We translate into hex and get 0x75 and 0x3 i.e. USART_BRR = 0x00000753
	*/
	USART3->BRR  = 0xEA0;
	USART3->CR1 &= ~USART_CR1_M; //8 data bits
	USART3->CR1 &= ~(USART_CR1_PS | USART_CR1_PCE); //no parity 
	USART3->CR2 &= ~USART_CR2_STOP; // 1 stop bit (STOP[13:12] = 00
	USART3->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE); //CTS and RTS hardware flow control disabled
  USART3->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; /*8 data bit, 1 start bit, 1 stop bit, no parity, reception mode*/
	
	NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority( 0, 1, 0 ));
	NVIC_EnableIRQ(USART3_IRQn);
	
	#endif // (USART_INTRF_IN_USE == USART_INTRF3)
}

void put_char(uint16_t c)
{
	//if(c=='\r') putchar('\n');
	
	#if (USART_INTRF_IN_USE == USART_INTRF1)
	while( !(USART1->SR &USART_SR_TXE) ){}; // wait until we are able to transmit
	// Transmit data
	USART1->DR = (c & 0xFF); // transmit the character
	#endif

	#if (USART_INTRF_IN_USE == USART_INTRF2)
	while( !(USART2->SR & USART_SR_TXE) ){}; // wait until we are able to transmit
	// Transmit data
	USART2->DR = (c & 0xFF); // transmit the character
	#endif
			
	#if (USART_INTRF_IN_USE == USART_INTRF3)
	while( !(USART3->SR &USART_SR_TXE) ){}; // wait until we are able to transmit
	// Transmit data
	USART3->DR = (c & 0xFF); // transmit the character
	#endif
}

__inline uint8_t put_str(unsigned char *s)
{
	uint8_t i = 0;
	while(s[i])
	{
		put_char(s[i]);
		i++;
	}
	put_char('\n');
	return i;
}

__inline uint16_t get_char(void)
{
	#if (USART_INTRF_IN_USE == USART_INTRF1)
	
	  while( !( USART1->SR & USART_SR_RXNE ) ) {}; // wait until something received
		return USART1->DR & 0x01FF; // find out what it is
	
	#endif // #if (USART_INTRF_NUMBER == USART_INTRF1)
	
	#if (USART_INTRF_IN_USE == USART_INTRF2)
	
    while( !( USART2->SR & USART_SR_RXNE ) ) {}; // wait until something received
		return USART2->DR & 0x01FF; // find out what it is
			
	#endif // #if (USART_INTRF_NUMBER == USART_INTRF2)
			
	#if (USART_INTRF_IN_USE == USART_INTRF3)
			
		while( !( USART3->SR & USART_SR_RXNE ) ) {}; // wait until something received
		return USART3->DR & 0x01FF; // find out what it is
			
	#endif // #if (USART_INTRF_NUMBER == USART_INTRF3)
}

static uint16_t compute_uart_bd(uint32_t p_clk, uint32_t bd_r)
{
	return ((p_clk + (bd_r/2U))/bd_r);
}

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t p_clk, uint32_t bd_r)
{
	USARTx->BRR = compute_uart_bd(p_clk, bd_r);
}

#if (USART_INTRF_IN_USE == USART_INTRF1)
extern void USART1_IRQHandler(void)
{
  if( USART1->SR & USART_SR_RXNE ) 
	{
		char c = USART1->DR;
		rbuf_write(rb, c);
		if( c == '\r' ) { newline = 1; }
	}
}
#endif // #if (USART_INTRF_NUMBER == USART_INTRF1)

#if (USART_INTRF_IN_USE == USART_INTRF2)
extern void USART2_IRQHandler(void)
{
	if( USART2->SR & USART_SR_RXNE ) 
	{
		char c = USART2->DR;
//		rbuf_write(rb, c);
//		if( c == '\r' ) { newline = 1; }
	}
}
#endif // #if (USART_INTRF_NUMBER == USART_INTRF2)

#if (USART_INTRF_IN_USE == USART_INTRF3)
extern void USART3_IRQHandler(void)
{
	if( USART3->SR & USART_SR_RXNE ) 
	{
		char c = USART3->DR;
		rbuf_write(rb, c);
		if( c == '\r' ) { newline = 1; }
	}
}
#endif // #if (USART_INTRF_NUMBER == USART_INTRF3)

