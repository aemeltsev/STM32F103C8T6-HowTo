#ifndef STM32F10x_USART_H
#define STM32F10x_USART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdbool.h>
#include <stdint.h>

#include "rbuf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
// USART interface numbers
#define USART_INTRF1												(1)
#define USART_INTRF2												(2)
#define USART_INTRF3												(3)

// Library configuration
// Select USART number
#define USART_INTRF_IN_USE							(USART_INTRF2)
#define USART_BAUDRATE                  (9600)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern rbuf rb;
extern volatile uint8_t newline;

/* Private function prototypes -----------------------------------------------*/
extern void USART_Init(void);
int _write(int handle, char* data, int size);

#if (USART_INTRF_NUMBER == USART_INTRF1)
extern void USART1_IRQHandler(void);
#endif // #if (USART_INTRF_NUMBER == USART_INTRF1)

#if (USART_INTRF_NUMBER == USART_INTRF2)
extern void USART2_IRQHandler(void);
#endif // #if (USART_INTRF_NUMBER == USART_INTRF2)

#if (USART_INTRF_NUMBER == USART_INTRF3)
extern void USART3_IRQHandler(void);
#endif // #if (USART_INTRF_NUMBER == USART_INTRF3)

#endif //