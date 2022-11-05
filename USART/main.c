//#include "stm32f10x.h" 
#include <stdio.h>
#include <stdint.h>
#include "USARTPool.h"

volatile char rb_buf[ RINGBUF_SIZE + 1 ];

rbuf rb = {
	.len = RINGBUF_SIZE,
	.buf = rb_buf,
	.pos = 0,
	.ext = 0
};

volatile uint8_t newline = 0;

int main(void){
	
	USART_Init();
	while(1)
	{
		while( newline == 0 )
		{
			break;
		}		
		while( rb.pos != rb.ext )
		{
			putchar(rbuf_read( &rb ) );
		}
		printf( "\n" );
		newline = 0;
	}
}