#include "stm32f10x.h" 
#include <stdio.h>
#include <stdint.h>
#include "usart.h"
#include "rbuf.h"

volatile char rb_buf[RBUF_SIZE+1];

rbuf rb = {
	.len = RBUF_SIZE,
	.buf = rb_buf,
	.pos = 0,
	.ext = 0
};

volatile int newline = 0;

void __attribute__((always_inline)) delay(uint32_t delay)
{
   while(delay--) __asm("");
}

int main(void)
{
	usart_init();
	while(1)
		{
			//int c = get_char();
			//put_char(c);
			//if(c == '\r') put_char('\n');
			//put_str("UART stands for “Universal Asynchronous Receiver / Transmitter”, and it is a very simple serial communication interface. In its most basic form, it only uses two data signals: “Receive” (RX) and “Transmit” (TX).");

			while(newline == 0)
				{
					__WFI();
				}
				
			while( rb.pos != rb.ext )
				{
					putchar(rbuf_read(&rb));
				}
			
				printf( "\n" );
				newline = 0;
		}
	}