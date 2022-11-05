#ifndef RINGBUF_H
#define RINGBUF_H

#define RINGBUF_SIZE (128)

typedef struct{
	int len;
	volatile char* buf;
	volatile int pos;
	volatile int ext;
}rbuf;

#define rbuf_write(rb, x) \
  rb.buf[ rb.ext ] = x; \
  if ( ( rb.ext + 1 ) >= rb.len ) { rb.ext = 0; } \
	else { rb.ext = rb.ext + 1; }
	
static inline char rbuf_read( rbuf* buffer ) 
{
	if( buffer->pos == buffer->ext) { return '\0'; }
	char read = buffer->buf[ buffer->pos ];
	buffer->pos = ( buffer->pos < ( buffer->len - 1 ) ) ? ( buffer->pos + 1 ) : 0;
	
	return read;
}

#endif //#define RINGBUF_H