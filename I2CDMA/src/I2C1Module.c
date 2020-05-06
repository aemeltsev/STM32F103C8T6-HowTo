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
* @brief  Initializes peripherals: I2C1.
*         Fclk = 8MHz => Tmaster = 0.125us
*         Standart Mode, 100KHz
*         Fi2c = 100KHz => Period(I2C) = 10us
*         For StandartMode Trise = 1000ns
*         Configure output pins.
*         SCL - pin42 - PB6
*         SDA - pin43 - PB7
* @param  None.
* @retval None.
*/
void I2C1_ConfInit(void)
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
* @brief  Initialize all peripherals.
* @param  None.
* @retval None.
*/
void I2C1_Init(void)
{
	I2C1_GpioInit();
	I2C1_ConfInit();
	I2C1_DmaInit();
}


/**
* @brief  Transmit(write) data _byte_ using polling.
* @param  regaddr - the address of internal register to write.
* @param  data - the data to write.
* @retval Success if data of writed.
*/
bool I2C_TX(uint16_t regaddr, uint8_t data)
{
	bool success = 1;                              /*check transmit byte data*/
	/*Start*/
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
	
	/*check SR2 and go on if OK*/
	if((I2C1->SR2 & I2C_SR2_MSL) && (I2C1->SR2 & I2C_SR2_BUSY))    /*master mode && communication ongoing*/
	{
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

        while(!(I2C1->SR1 & I2C_SR1_BTF)){};     /*wait BFT - Byte transfer finished*/
				I2C1->CR1 |= I2C_CR1_STOP;               /*Program the STOP bit*/
				while (I2C1->CR1 & I2C_CR1_STOP);        /*Wait until STOP bit is cleared by hardware*/
	}
	else
	{
		success = 0;
	}
	return success;
}

/**
* @brief  Transmit(write) a page.
* @param  regaddr - the address of internal register to write.
* @param  data_ptr - the pointer to data for write.
* @param  data_length.
* @retval Success if data of writed.
*/
bool I2C_TX_PAGE(uint16_t regaddr, uint8_t *data_ptr, uint16_t data_length)
{
	bool success = 1;
	
	/*Start*/
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
	
	/*check SR2 and go on if OK*/
	if((I2C1->SR2 & I2C_SR2_MSL) && (I2C1->SR2 & I2C_SR2_BUSY))    /*master mode && communication ongoing*/
	{
				/*Send the high byte of internal address for write*/
		    I2C1->DR=(uint8_t) (regaddr >> 8);
		    /*Test on EV8*/
        while (!(I2C1->SR1 & I2C_SR1_TXE)){};
				
				/*Send the low byte of internal address for write*/
				I2C1->DR=(uint8_t)(regaddr & 0x00FF);
				/*Test on EV8*/
        while (!(I2C1->SR1 & I2C_SR1_TXE)){};
				
				while(data_length > 0)
				{
					   /*Send the byte to be written*/
				     I2C1->DR = *data_ptr;
					            
					   data_ptr++;
					   data_length--;
					
             while(!(I2C1->SR1 & I2C_SR1_BTF)){};     /*wait BFT - Byte transfer finished*/
				}
				I2C1->CR1 |= I2C_CR1_STOP;               /*Program the STOP bit*/
				while (I2C1->CR1 & I2C_CR1_STOP);        /*Wait until STOP bit is cleared by hardware*/
	}
	else
	{
		success = 0;
	}
	return success;
}

/**
* @brief  Receive(read) data byte.
* @param  regaddr - the address of internal register to read.
* @retval Byte of read data.
*/
uint8_t I2C_RX(uint16_t regaddr)
{
	uint8_t data;
	/*Start*/
	I2C1->CR1 |= I2C_CR1_START;                    /*START, = 0x0100*/
	
	/*Test on EV5 and clear it*/
	while(!(I2C1->SR1 & I2C_SR1_SB)){};            /*wait SB, while(!(I2C1->SR1 & 0x0001))*/
		(void)I2C1->SR1;                             /*clear SB*/
		
	/*ADDR*/
	I2C1->DR = I2C_SLAVE_ADDR;
	/*Test on EV6 and clear it*/
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){};          /*wait ADDR, while(!(I2C1->SR1 & 0x0002))*/
  (void) I2C1->SR1;                              /*2.reset ADDR*/
  (void) I2C1->SR2;                              /*2.reset ADDR */
	
	/*Send the high byte of internal address for read*/
	I2C1->DR=(uint8_t) (regaddr>>8);
	/*Test on EV8*/
  while (!(I2C1->SR1 & I2C_SR1_TXE)){};
	
	/*Send the low byte of internal address for read*/
  I2C1->DR=(uint8_t)(regaddr &0x00FF);
	/*Test on EV8*/
  while (!(I2C1->SR1 & I2C_SR1_TXE)){};
	
	/*Restart*/
	I2C1->CR1 |= I2C_CR1_START;
	
	/*Test on EV5 and clear it*/
	while(!(I2C1->SR1 & I2C_SR1_SB)){};            /*wait SB, while(!(I2C1->SR1 & 0x0001))*/
		(void)I2C1->SR1;                             /*clear SB*/

	/*ADDR*/
	I2C1->DR = I2C_SLAVE_ADDR | 0x01;
	/*Test on EV6 and clear it*/
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){};          /*wait ADDR, while(!(I2C1->SR1 & 0x0002))*/
  (void) I2C1->SR1;                              /*2.reset ADDR*/
  (void) I2C1->SR2;                              /*2.reset ADDR */
	
	/*NACK*/
	I2C1->CR1 &= ~I2C_CR1_ACK;
	while(!(I2C1->SR1 & I2C_SR1_RXNE));            /*wait RxNE, while(!(I2C1->SR1 & 0x0040))*/
	
	/*Read data*/
	data = I2C1->DR;
	I2C1->CR1 |= I2C_CR1_STOP;                     /*Generate STOP*/
	return data;
}

/**
* @brief  Receive(read) data byte.
* @param  regaddr - the address of internal register to read.
* @param  data_ptr - the pointer to data for read.
* @param  data_length.
* @retval Success if data of read.
*/
bool I2C_RX_PAGE(uint16_t regaddr, uint8_t *data_ptr, uint16_t data_length)
{
	bool success = 1;
	
	/*Start*/
	I2C1->CR1 |= I2C_CR1_START;                    /*START, = 0x0100*/
	
	/*Test on EV5 and clear it*/
	while(!(I2C1->SR1 & I2C_SR1_SB)){};            /*wait SB, while(!(I2C1->SR1 & 0x0001))*/
		(void)I2C1->SR1;                             /*clear SB*/
		
	/*ADDR*/
	I2C1->DR = I2C_SLAVE_ADDR;
	/*Test on EV6 and clear it*/
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){};          /*wait ADDR, while(!(I2C1->SR1 & 0x0002))*/
  (void) I2C1->SR1;                              /*2.reset ADDR*/
  (void) I2C1->SR2;                              /*2.reset ADDR */
	
	/*check SR2 and go on if OK*/
	if((I2C1->SR2 & I2C_SR2_MSL) && (I2C1->SR2 & I2C_SR2_BUSY))    /*master mode && communication ongoing*/
	{
		/*Send the high byte of internal address for read*/
		I2C1->DR=(uint8_t) (regaddr>>8);
		/*Test on EV8*/
    while (!(I2C1->SR1 & I2C_SR1_TXE)){};
			
		/*Send the low byte of internal address for read*/
		I2C1->DR=(uint8_t)(regaddr &0x00FF);
		/*Test on EV8*/
    while (!(I2C1->SR1 & I2C_SR1_TXE)){};

    /*Restart*/
	  I2C1->CR1 |= I2C_CR1_START;
	
	  /*Test on EV5 and clear it*/
	  while(!(I2C1->SR1 & I2C_SR1_SB)){};            /*wait SB, while(!(I2C1->SR1 & 0x0001))*/
		(void)I2C1->SR1;                               /*clear SB*/

	  /*ADDR*/
	  I2C1->DR = I2C_SLAVE_ADDR | 0x01;
	  /*Test on EV6 and clear it*/
	  while(!(I2C1->SR1 & I2C_SR1_ADDR)){};          /*wait ADDR, while(!(I2C1->SR1 & 0x0002))*/
    (void) I2C1->SR1;                              /*2.reset ADDR*/
    (void) I2C1->SR2;                              /*2.reset ADDR */
		
		if(I2C1->SR2 & I2C_SR2_BUSY)
		{
			I2C1->CR1 |= I2C_CR1_ACK;                    /*ACK*/
			
			while(data_length > 0)
			{
				while(!(I2C1->SR1 & I2C_SR1_RXNE));        /*wait RxNE, while(!(I2C1->SR1 & 0x0040))*/
				
				/*Read data*/
				*data_ptr = I2C1->DR;
				
				data_ptr++;
				data_length--;
				
				if(data_length == 1)
				{
					I2C1->CR1 &= ~I2C_CR1_ACK;             /*NACK*/
				}
			}
			I2C1->CR1 |= I2C_CR1_STOP;                 /*Generate STOP*/
		}
		else
		{
			success = 0;
		}
			
	}	
	else
	{
		success = 0;
	}
	return success;
}

/**
* @brief  Recieve(read) with DMA.
* @param  regaddr - the address of internal register to read.
* @param  data_ptr - the pointer to data for read.
* @param  data_length - number of data to transfer.
* @param  stop_flag.
* @retval None.
*/
void I2C_DMA_RX(uint16_t regaddr, uint8_t *data_ptr, uint16_t data_length, uint8_t stop_flag)
{	
	I2C1->CR2 |= I2C_CR2_ITERREN;                  /*Enable iterrupt for I2C error*/
	//I2C1->CR1 |= I2C_CR1_ACK;                      /*Access recive*/
	I2C1->CR2 |= I2C_CR2_DMAEN;                    /*Enable request I2C=>DMA*/
	I2C1->CR2 |= I2C_CR2_LAST;                     /*Enable DMA last transfer*/
	
	/*Start*/
	I2C1->CR1 |= I2C_CR1_START;                    /*START, = 0x0100*/
	
	/*Test on EV5 and clear it*/
	while(!(I2C1->SR1 & I2C_SR1_SB)){};            /*wait SB, while(!(I2C1->SR1 & 0x0001))*/
	(void)I2C1->SR1;                               /*clear SB*/
	
	/*ADDR*/
	I2C1->DR = I2C_SLAVE_ADDR;
	/*Test on EV6 and clear it*/
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){};          /*wait ADDR, while(!(I2C1->SR1 & 0x0002))*/
  (void) I2C1->SR1;                              /*2.reset ADDR*/
  (void) I2C1->SR2;                              /*2.reset ADDR */
			
	/*Channel 7 DMA*/
	DMA1_Channel7->CMAR = (uint32_t)(data_ptr);    /*Address recieve array*/
	DMA1_Channel7->CNDTR = data_length;            /*Number recieve byte*/
	DMA1_Channel7->CCR |= DMA_CCR1_EN;             /*Enable Channel 7 DMA*/
	
	/*wait to transfer Complete flag*/
	while(!(DMA1->ISR & DMA_ISR_TCIF7)) {continue;}
	
	DMA1->IFCR = DMA_IFCR_CTCIF7;                  /*!< Channel 7 Transfer Complete clear */
	
	/*Test on EV8*/
	while(!(I2C1->SR1 & I2C_SR1_RXNE)) {continue;}
	
  I2C1->CR2 &= ~I2C_CR2_DMAEN;                   /*!< Close request I2C=>DMA */
	DMA1_Channel7->CCR &= ~DMA_CCR1_EN;            /*!< Close channel 7 DMA */

	if(stop_flag)
	{
		I2C1->CR1 |= I2C_CR1_STOP;                     /*Program the STOP bit*/
		while (I2C1->CR1 & I2C_CR1_STOP);              /*Wait until STOP bit is cleared by hardware*/
	}
	
	I2C1->CR2 &= ~I2C_CR2_ITERREN;                 /*!< Close itterupt I2C=>DMA */
		
}

/**
* @brief  Transmit(write) with DMA.
* @param  regaddr - the address of internal register to write.
* @param  data_ptr - the pointer to data for write.
* @param  data_length - number of data to transfer.
* @param  stop_flag.
* @retval None.
*/
void I2C_DMA_TX(uint16_t regaddr, uint8_t *data_ptr, uint16_t data_length, uint8_t stop_flag)
{
	I2C1->CR2 |= I2C_CR2_ITERREN;                  /*Enable iterrupt for I2C error*/
	//I2C1->CR1 |= I2C_CR1_ACK;                    /*Access recive*/
	I2C1->CR2 |= I2C_CR2_DMAEN;                    /*Enable request I2C=>DMA*/
	I2C1->CR2 |= I2C_CR2_LAST;                     /*Enable DMA last transfer*/
	
	/*Start*/
	I2C1->CR1 |= I2C_CR1_START;                    /*START, = 0x0100*/
	
	/*Test on EV5 and clear it*/
	while(!(I2C1->SR1 & I2C_SR1_SB)){};            /*wait SB, while(!(I2C1->SR1 & 0x0001))*/
	(void)I2C1->SR1;                               /*clear SB*/
	
	/*ADDR*/
	I2C1->DR = I2C_SLAVE_ADDR;
	/*Test on EV6 and clear it*/
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){};          /*wait ADDR, while(!(I2C1->SR1 & 0x0002))*/
  (void) I2C1->SR1;                              /*2.reset ADDR*/
  (void) I2C1->SR2;                              /*2.reset ADDR */
	
	/*Channel 7 DMA*/
	DMA1_Channel6->CMAR = (uint32_t)(data_ptr);    /*Address transmit array*/
	DMA1_Channel6->CNDTR = data_length;            /*Number transmit byte*/
	DMA1_Channel6->CCR |= DMA_CCR1_EN;             /*Enable Channel 6 DMA*/
	
	/*wait to transfer Complete flag*/
	while(!(DMA1->ISR & DMA_ISR_TCIF6)) {continue;}
	
	DMA1->IFCR = DMA_IFCR_CTCIF6;                  /*Channel 6 Transfer Complete clear*/
	
	/*Byte Transfer Finished*/
	while(I2C1->SR1 & I2C_SR1_BTF) {continue;}
	
	I2C1->CR2 &= ~I2C_CR2_DMAEN;                   /*Close request I2C=>DMA*/
	DMA1_Channel6->CCR &= ~DMA_CCR1_EN;            /*Close channel 7 DMA*/
	
	if(stop_flag)
	{
		I2C1->CR1 |= I2C_CR1_STOP;                   /*Program the STOP bit*/
		while (I2C1->CR1 & I2C_CR1_STOP);            /*Wait until STOP bit is cleared by hardware*/
	}
	
	I2C1->CR2 &= ~I2C_CR2_ITERREN;                 /*Close itterupt I2C=>DMA*/
}
	