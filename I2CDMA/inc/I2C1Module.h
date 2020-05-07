/**
  ******************************************************************************
  * @file    I2C1Module.h
  * @author  Anthony Emeltsev
  * @version V0.2
  * @date    May-2020
  * @brief   STM32F10X, I2C, DMA, Master Mode.
	*
	* The MIT License (MIT)
  *
  * Copyright (c) 2020 Anthony Emeltsev
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  * SOFTWARE.
  ******************************************************************************
  */
	
#include "stm32f10x.h"

/* Define data and control lines  */
#define LAST    ((uint8_t)0x01)
#define NOLAST  ((uint8_t)0x00)
#define I2C_TIMEOUT_BUSY  50     // Timeout I2C process
#define I2C_ERR_BUSY     128     // Error busy
#define I2C_SET_CLK_FRQ    8     // Peripheral clock frequency
#define I2C_SET_CCR       40     // CCR=Period(I2C)/(2xTmaster) (for StandartMode)
#define I2C_SET_TRISE      9     // TRISER=(Trise/Tmaster)+1=(1/0.125)+1=9

#define I2C_SLAVE_ADDR   0xD0

/**
 ***************************************************************
 *
 * FUNCTIONS
 *
 ***************************************************************
 */
void I2C_delay(int32_t volatile DelayTime_uS);

void I2C1_GpioInit(void);
void I2C1_ConfInit(void);
void I2C1_DmaInit(void);
void I2C1_Init(void);

bool I2C_TX(uint16_t regaddr, uint8_t data);
bool I2C_TX_PAGE(uint16_t regaddr, uint8_t *data_ptr, uint16_t data_length);
uint8_t I2C_RX(uint16_t regaddr);
bool I2C_RX_PAGE(uint16_t regaddr, uint8_t *data_ptr, uint16_t data_length);
void I2C_DMA_RX(uint16_t regaddr, uint8_t *data_ptr, uint16_t data_length, uint8_t stop_flag);
void I2C_DMA_TX(uint16_t regaddr, uint8_t *data_ptr, uint16_t data_length, uint8_t stop_flag);

