#include "stm32f10x.h" 
#include <stdio.h>
#include <stdint.h>

void delay(unsigned int time) {
	for (time; time > 0; time--);
}

int main(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	
	GPIOC->CRH |= GPIO_CRH_MODE13;
	
	GPIOC->CRH &= ~(GPIO_CRH_CNF13);
	
	while(1)
	{
		GPIOC->BSRR = GPIO_BSRR_BS13;
		delay(550000);
		GPIOC->BSRR =GPIO_BSRR_BR13;
		delay(550000);
	}
	 
}