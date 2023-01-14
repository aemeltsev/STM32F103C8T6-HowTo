#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>

#define RBUF_SIZE (128)

typedef struct
{
	volatile uint8_t* buffer;
	volatile int head;
	volatile int tail;
	int len;
}circ_bbuf_t;

static void rbuf_write(circ_bbuf_t* buffer, uint8_t c)
{
	buffer->buffer[buffer->tail] = c;
	if((buffer->tail + 1) >= buffer->len) { buffer->tail = 0;}
	else {buffer->tail = buffer->tail + 1;}
}

static inline uint8_t rbuf_read(circ_bbuf_t* buffer) 
{
   if(buffer->head == buffer->tail) {return -1;}
	 int read = buffer->buffer[ buffer->head ];
	 buffer->head = ( buffer->head < ( buffer->len - 1 ) ) ? ( buffer->head + 1 ) : 0;

	 return read;
}

#endif //#define RINGBUF_H