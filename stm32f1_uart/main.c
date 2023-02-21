#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

static void init_clock(void)
{
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_24MHZ]);

	/* Enable GPIOC clock (for LED GPIOs). */
	rcc_periph_clock_enable(RCC_GPIOC);

	// Enable clocks for GPIO port A: GPIO_USART1_TX, USART1
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);
}

static void init_usart(void)
{
	//////////////////////////////////////////////////////////////
	// STM32F103C8T6:
	//	RX:	A9
	//	TX:	A10
	//	CTS:	A11 (not used)
	//	RTS:	A12 (not used)
	//	Baud:	38400
	//////////////////////////////////////////////////////////////

	/* Setup GPIO9 (in GPIO port A) to 'output push-pull' for LED use. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	//gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
	//              GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 38400);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX);
	//usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	usart_enable(USART1);
}

static void init_gpio(void)
{	
	/* Set GPIO8 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		          GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

int main(void)
{
	int i, j = 0, c = 0;

	init_clock();
	init_gpio();
	init_usart();

	for (;;)
	{
		gpio_toggle(GPIOC, GPIO13);	// Toggle LED
		usart_send_blocking(USART1, c + '0'); /* Send a byte. */
		c = (c == 9) ? 0 : c + 1; /* Increment c. */
		if ((j++ % 80) == 0) /* Newline after line full. */
		{
			usart_send_blocking(USART1, '\r');
			usart_send_blocking(USART1, '\n'); 
		}
		for(i = 0; i < 800000; i++){__asm__("nop");} /* Wait a bit. */
	}
	return 0;
}
