/**
  ******************************************************************************
  * @file    I2C1Module.c
  * @author  IRQ
  * @version V0.1
  * @date    March-2019
  * @brief   Master Mode, 7-bit Address.
  ******************************************************************************
  */

#include "I2C1Module.h"

uint8_t I2C_Error;               // Error I2C module
uint8_t I2C_Timer;               // ?????? ?????? ?????? I2C

/**
* @brief  Set delay time.
* @param  DelayTime_uS - in us.
* @retval None.
*/
void I2C_delay(int32_t volatile DelayTime_uS)
{
	uint32_t DelayTime = 0;
	DelayTime = (SystemCoreClock/1000000)*DelayTime_uS;
	for (; DelayTime!=0; DelayTime--);
}

/**
* @brief  Initializes peripherals: GPIOB.
* @param  None.
* @retval None.
*/
void I2C1_GpioInit(void)
{
	/*Clock enable*/
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;    /*Enable clocking PortB*/
	
	/*Configuration GPIOB.6*/
	GPIOB->CRL &= ~GPIO_CRL_MODE6;   /*clear bits MODE*/
	GPIOB->CRL &= ~GPIO_CRL_CNF6;    /*clear bits CNF*/
	GPIOB->CRL |=  GPIO_CRL_MODE6_1; /*output, 2MHz*/
	GPIOB->CRL |=  GPIO_CRL_CNF6;    /*alternate function, open drain*/
	
	/*Configuration GPIOB.7*/
	GPIOB->CRL &= ~GPIO_CRL_MODE7;   /*clear bits MODE*/
	GPIOB->CRL &= ~GPIO_CRL_CNF7;    /*clear bits CNF*/
	GPIOB->CRL |=  GPIO_CRL_MODE7_1; /*output, 2MHz*/
	GPIOB->CRL |=  GPIO_CRL_CNF7;    /*alternate function, open drain*/
}

/**
* @brief Initializes peripherals: I2C1.
* Fclk = 8MHz => Tmaster = 0.125us
* Standart Mode, 100KHz
* Fi2c = 100KHz => Period(I2C) = 10us
* For StandartMode Trise = 1000ns
* Configure output pins.
* SCL - pin42 - PB6
* SDA - pin43 - PB7
* @param  None.
* @retval None.
*/
void I2C1_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;    /*Enable clocking I2C1 module*/
	
	/* Configuration I2C1 */
	I2C2->CR1 &= ~I2C_CR1_SMBUS;     /*Use I2C*/
	
	I2C1->CR2 &= ~I2C_CR2_FREQ;      /*Reset peripheral clock frequency*/
	I2C1->CR2 |= I2C_SET_CLK_FRQ;    /*Clock frequency 8Mhz*/
	
	I2C1->CCR &= ~I2C_CCR_CCR;       /*Reset clock control register*/
	I2C1->CCR |= I2C_SET_CCR;        /*CCR=Period(I2C)/(2xTmaster) (for StandartMode)*/
	
	I2C1->TRISE = I2C_SET_TRISE;     /*TRISER=(Trise/Tmaster)+1=(1/0.125)+1=9*/
	I2C1->CCR |= I2C_CCR_DUTY;       /*Fast Mode Duty Cycle - 1: tlow/thigh = 16/9*/
	
	//I2C1->CR2 |= I2C_CR2_LAST;       /*DMA Last Transfer*/
	I2C1->CR1 |= I2C_CR1_PE;         /*Enable I2C module*/
}

/**
* @brief  Initializes peripherals: DMA1 channels.
* @param  None.
* @retval None.
*/
void I2C1_DmaInit(void)
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;      /*Enable clocking DMA1*/
	
	/*Configuration channel 7 DMA - I2C1_RX*/
	DMA1_Channel7->CCR &= ~DMA_CCR7_CIRC;                         /*Not circular mode*/
	DMA1_Channel7->CCR |= DMA_CCR7_PL;                            /*Very high channel priority level*/
	DMA1_Channel7->CCR &= ~DMA_CCR7_MEM2MEM;                      /*Memory to memory mode disable*/
	DMA1_Channel7->CCR |= DMA_CCR7_MINC;                          /*Memory increment mode*/
	DMA1_Channel7->CCR &= ~DMA_CCR7_DIR;                          /*Data transfer direction read from peripheral*/
	DMA1_Channel7->CPAR = (*((volatile uint32_t *)I2C1->DR));     /*Address I2C1_DR*/
	
	/*Configuration channel 6 DMA - I2C1_TX*/
	DMA1_Channel6->CCR &= ~DMA_CCR6_CIRC;                         /*Not circular mode*/
	DMA1_Channel6->CCR |= DMA_CCR6_PL;                            /*Very high channel priority level*/
	DMA1_Channel6->CCR &= ~DMA_CCR6_MEM2MEM;                      /*Memory to memory mode disable*/
	DMA1_Channel6->CCR |= DMA_CCR6_MINC;                          /*Memory increment mode*/
	DMA1_Channel6->CCR &= ~DMA_CCR6_DIR;                          /*Data transfer direction read from peripheral*/
	DMA1_Channel6->CPAR = (*((volatile uint32_t *)I2C1->DR));     /*Address I2C1_DR*/   
}

/**
* @brief  Initializes peripherals: DMA1 channels.
* @param  None.
* @retval None.
*/


/**
* @brief  Set value Start.
  * @param  None.
  * @retval None.
  */
void I2C_Start(void)
{
	I2C_Error = 0x00;
	I2C_Timer = I2C_TIMEOUT_BUSY;
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){
		
		if(!I2C_Timer){
			I2C_Error = I2C_ERR_BUSY;
			break;
		}
	}
}

/**
* @brief  Set value Stop.
* @param  None.
* @retval None.
*/
void I2C_Stop(void)
{
	I2C1->CR1 |= I2C_CR1_STOP;
	while((I2C1->SR1 & I2C_SR1_STOPF) == (uint16_t)0x0010);
}

/**
* @brief  Generate NACK.
* @param  None.
* @retval None.
*/
void I2C_Nack(void)
{
	I2C1->CR1 &= ~I2C_CR1_ACK;
}

/**
* @brief  Transmit address slave.
* @param  Transmit address(7 bits + bit read/write).
* @retval None.
*/
void I2C_Address(uint8_t I2C_Temp)
{
	I2C1->DR = I2C_Temp;
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){
		
		if(!I2C_Timer){
			I2C_Error = I2C_ERR_BUSY;
			break;
		}
	}
	I2C_Temp = I2C1->SR2;
}

/**
* @brief  Transmit(write) data byte using polling.
* @param  regaddr - the address of internal register to write.
* @param  data - the data to write.
* @param  last - number of bytes to write.
* @retval None.
*/
void I2C_TX(uint8_t regaddr, uint8_t data)
{
	I2C1->CR1 |= I2C_CR1_START;                    /*START, = 0x0100*/
	
	/*Test on EV5 and clear it*/
	while(!(I2C1->SR1 & I2C_SR1_SB)){};            /*wait SB, while(!(I2C1->SR1 & 0x0001))*/
		(void)I2C1->SR1;                             /*clear SB*/
	
	/*ADDR*/
	I2C1->DR = I2C_SLAVE_ADDR;		                 /*Send slave address*/
	/*Test on EV6 and clear it*/
	while (!(I2C1->SR1 & I2C_SR1_ADDR)){}    	     /*wait ADDR*/
	(void) I2C1->SR1;                              /*reset ADDR*/
  (void) I2C1->SR2;                              /*reset ADDR*/
		
	/*Send the high byte of internal address for write*/
	I2C1->DR=(uint8_t) (regaddr >> 8);
	/*Test on EV8*/
	while (!(I2C1->SR1 & I2C_SR1_TXE)){};
		
	/*Send the low byte of internal address for write*/
	I2C1->DR=(uint8_t)(regaddr & 0x00FF);
	/*Test on EV8*/
	while (!(I2C1->SR1 & I2C_SR1_TXE)){};
		
	/*Send the byte to be written*/
	I2C1->DR=data;
	
	while(!(I2C1->SR1 & I2C_SR1_BTF)){};       /*wait BFT - Byte transfer finished*/
	I2C1->CR1 |= I2C_CR1_STOP;                 /*Program the STOP bit*/
	while (I2C1->CR1 & I2C_CR1_STOP);          /*Wait until STOP bit is cleared by hardware*/
}

/**
* @brief  Receive(read) data byte.
Function from this https://gitlab.com/flank1er/stm32_bare_metal/tree/master
  * @param  None.
  * @retval None.
  */
uint8_t I2C_ByteRX(uint8_t adr)
{
	uint8_t data;
	I2C1->CR1 |= I2C_CR1_START;                    /*!< START, =0x0100 */
	while(!(I2C1->SR1 & I2C_SR1_SB))               /*!< wait SB, while(!(I2C1->SR1 & 0x0001)) */
	(void)I2C1->SR1;                               /*!< clear SB */
	I2C1->DR = adr;                                /*!< wait ADDR, while(!(I2C1->SR1 & 0x0002)) */
	while(!(I2C1->SR1 & I2C_SR1_ADDR));
	I2C1->CR1 &= ~I2C_CR1_ACK;                     /*!< 1.NACK, =0xFBFF */
	__disable_irq();
  (void) I2C1->SR1;                              /*!< 2.reset ADDR */
  (void) I2C1->SR2;                              /*!< 2.reset ADDR */
	I2C1->CR1 |= I2C_CR1_STOP;                     /*!< 3.STOP, =0x0200 */
	__enable_irq();
	while(!(I2C1->SR1 & I2C_SR1_RXNE));            /*!< wait RxNE, while(!(I2C1->SR1 & 0x0040)) */
	data = I2C1->DR;
	while(I2C1->CR1 & I2C_CR1_STOP);               /*!< wait clear STOP */
	I2C1->CR1 |= I2C_CR1_ACK;                      /*!< ACK for next byte */
	return data;
}

/**
* @brief  Receive(read) two data byte.
Function from this https://gitlab.com/flank1er/stm32_bare_metal/tree/master
  * @param  None.
  * @retval None.
  */
uint16_t I2C_TwoByteRX(uint8_t adr)
{
	uint8_t data=0;
	uint8_t vl;
	I2C1->CR1 |= I2C_CR1_START;                    /*!< START, =0x0100 */
	while(!(I2C1->SR1 & I2C_SR1_SB))               /*!< wait SB, while(!(I2C1->SR1 & 0x0001)) */
	(void)I2C1->SR1;
	I2C1->DR = adr;
	while(!(I2C1->SR1 & I2C_SR1_ADDR));            /*!< wait ADDR, while(!(I2C1->SR1 & 0x0002)) */
	I2C1->CR1 |= I2C_CR1_POS;                      /*!< POS=1 */
	__disable_irq();
	(void) I2C1->SR1;                              /*!< reset ADDR */
  (void) I2C1->SR2;                              /*!< reset ADDR */
	I2C1->CR1 &= ~I2C_CR1_ACK;                     /*!< NACK, =0xFBFF */
	__enable_irq();
	while(!(I2C1->SR1 & I2C_SR1_BTF));             /*!< wait BFT */
	__disable_irq();
	I2C1->CR1 |= I2C_CR1_STOP;                     /*!< STOP, =0x0200 */
	vl = I2C1->DR;
	data |= (uint16_t)vl;
	__enable_irq();
	//while(!(I2C1->SR1 & I2C_SR1_RXNE));            /*!< wait RxNE, while(!(I2C1->SR1 & 0x0040)) */
	vl = I2C1->DR;
	data |= (uint16_t)(vl<<8);
	while(I2C1->CR1 & I2C_CR1_STOP);               /*!< wait clear STOP */
	I2C1->CR1 &= ~I2C_CR1_POS;                     /*!< POS=0 */
	I2C1->CR1 |= I2C_CR1_ACK;                      /*!< ACK for next byte */
	return data;
}

/**
* @brief  Recieve with DMA.
  * @param  I2C_Addr - address slave.
  * @param  *ptr - pointer to array input .
  * @param  I2C_NumByte - .
  * @param  I2C_StopFlag.
  * @retval None.
  */
void I2C_DMARx(uint8_t I2C_Addr, uint8_t *ptr, uint8_t I2C_NumByte, uint8_t I2C_StopFlag)
{
	/*!< Recieve */
	I2C_Error = 0x00;                              /*!< Reset error */
	I2C1->CR2 |= I2C_CR2_ITERREN;                  /*!< Enable iterrupt for I2C error */
	I2C_Start();
	  if(!I2C_Error) I2C_Address(I2C_Addr | 0x01);
	  if(!I2C_Error){
		  /*!< Recieve data */
		  I2C1->CR1 |= I2C_CR1_ACK;                  /*!< Access recive */
		  I2C1->CR2 |= I2C_CR2_DMAEN;                /*!< Enable request I2C=>DMA */
		  /*!< Channel 7 DMA */
		  DMA1_Channel7->CMAR = (uint32_t)(ptr);     /*!< Address recieve array */
		  DMA1_Channel7->CNDTR = I2C_NumByte;        /*!< Number recieve byte */
		  DMA1_Channel7->CCR |= DMA_CCR1_EN;         /*!< Enable Channel 7 DMA */
		    while(!(DMA1->ISR & DMA_ISR_TCIF7)) continue;
		  DMA1->IFCR = DMA_IFCR_CTCIF7;              /*!< Channel 7 Transfer Complete clear */
		    while(!(I2C1->SR1 & I2C_SR1_RXNE))continue;
		  I2C1->CR2 &= ~I2C_CR2_DMAEN;               /*!< Close request I2C=>DMA */
		  DMA1_Channel7->CCR &= ~DMA_CCR1_EN;        /*!< Close channel 7 DMA */
		}
	    if(I2C_StopFlag)I2C_Stop();
	I2C1->CR2 &= ~I2C_CR2_ITERREN;                 /*!< Close itterupt I2C=>DMA */
		
}

/**
* @brief  Transmit with DMA.
  * @param  I2C_Addr.
  * @param  *ptr.
  * @param  I2C_NumByte.
  * @param  I2C_StopFlag.
  * @retval None.
  */
void I2C_DMATx(uint8_t I2C_Addr, uint8_t *ptr, uint8_t I2C_NumByte, uint8_t I2C_StopFlag)
{
	/*!< Transmit */
	I2C_Error = 0x00;                              /*!< Reset error */
	I2C1->CR2 |= I2C_CR2_ITERREN;                  /*!< Enable iterrupt for I2C error */
	I2C_Start();
	  if(!I2C_Error) I2C_Address(I2C_Addr);
	  if(!I2C_Error){
			I2C1->CR2 |= I2C_CR2_DMAEN;                /*!< Enable request I2C=>DMA */
			/*!< Channel 7 DMA */
			DMA1_Channel6->CMAR = (uint32_t)(ptr);     /*!< Address transmit array */
			DMA1_Channel6->CNDTR = I2C_NumByte;        /*!< Number transmit byte */
			DMA1_Channel6->CCR |= DMA_CCR1_EN;         /*!< Enable Channel 6 DMA */
			  while(!(DMA1->ISR & DMA_ISR_TCIF6)) continue;
			DMA1->IFCR = DMA_IFCR_CTCIF6;              /*!< Channel 6 Transfer Complete clear */
			  while(I2C1->SR1 & I2C_SR1_BTF) continue;
			I2C1->CR2 &= ~I2C_CR2_DMAEN;               /*!< Close request I2C=>DMA */
			DMA1_Channel6->CCR &= ~DMA_CCR1_EN;        /*!< Close channel 7 DMA */
		}
		if(I2C_StopFlag) I2C_Stop();
	I2C1->CR2 %= ~I2C_CR2_ITERREN;                 /*!< Close itterupt I2C=>DMA */
}
	