/**
  ******************************************************************************
  * @file    I2C1Module.h
  * @author  IRQ
  * @version V0.1
  * @date    05-March-2019
  * @brief   Provides a library to access a HD44780-based character LCD module.
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

void I2C1_GpioInit(void);
void I2C1_Init(void);
void I2C1_DmaInit(void);

void I2C_Start(void);
void I2C_Address(uint8_t I2C_Temp);
void I2C_delay(int32_t volatile DelayTime_uS);
void I2C_TX(uint8_t regaddr, uint8_t data);
uint8_t I2C_ByteRX(uint8_t adr);
void I2C_Stop(void);
void I2C_DMARx(uint8_t I2C_Addr, uint8_t *ptr, uint8_t I2C_NumByte, uint8_t I2C_StopFlag);
void I2C_DMATx(uint8_t I2C_Addr, uint8_t *ptr, uint8_t I2C_NumByte, uint8_t I2C_StopFlag);

