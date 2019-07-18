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
#define I2C_TIMEOUT_BUSY  50     // Timeout I2C process
#define I2C_ERR_BUSY     128     // Error busy
#define I2C_SLAVE_ADDR   0xd0

/**
 ***************************************************************
 *
 * FUNCTIONS
 *
 ***************************************************************
 */

void I2C1_LowLevelInit(void);
void I2C_Start(void);
void I2C_Address(uint8_t I2C_Temp);
void I2C_ByteTX(uint8_t adr, uint8_t reg, uint8_t count, uint8_t* data);
uint8_t I2C_InitTX(uint8_t adr, uint8_t value, uint8_t last);
uint8_t I2C_ByteRX(uint8_t adr);
void I2C_Stop(void);
void I2C_DMARx(uint8_t I2C_Addr, uint8_t *ptr, uint8_t I2C_NumByte, uint8_t I2C_StopFlag);
void I2C_DMATx(uint8_t I2C_Addr, uint8_t *ptr, uint8_t I2C_NumByte, uint8_t I2C_StopFlag);

