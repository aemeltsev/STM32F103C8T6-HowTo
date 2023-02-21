#include <stdlib.h>
#include <string.h>
#include <libopencmsis/core_cm3.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static void gpio_setup(void)
{
	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOC);
	
	/* Set GPIO8 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

int main(void)
{
	int i;

	gpio_setup();

	/* Blink the LEDs on the board. */
	for (;;) {
		gpio_clear(GPIOC, GPIO13);      /* LED on */
		for (i = 0; i < 8000000; i++)	/* Wait a bit. */
			__asm__("nop");
		gpio_set(GPIOC, GPIO13);	/* LED on/off */
		for (i = 0; i < 8000000; i++)	/* Wait a bit. */
			__asm__("nop");
	}

	return 0;
}
