#ifndef _AVR_RINGBUFFER_H_
#define _AVR_RINGBUFFER_H_

#ifndef RING_BUFFER_SZ
#define RING_BUFFER_SZ  128
#endif

typedef struct
{
	volatile uint8_t headIdx;
	volatile uint8_t len;	
	volatile uint8_t bufferData[RING_BUFFER_SZ];
} RingBuffer;


void ringBufferInitialize(RingBuffer* r)
{
	r->headIdx = r->len = 0;
}


uint8_t ringBufferDepth(RingBuffer* r)
{
	return r->len;
}

uint8_t ringBufferPushNonBlocking(RingBuffer* r, uint8_t data)
{
	uint8_t res=0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if (r->len < RING_BUFFER_SZ)
		{
			r->bufferData[r->headIdx++] = data;
			r->len++;
			if( r->headIdx >= RING_BUFFER_SZ )
				r->headIdx = 0;
			res=1;
		}
	}
	return res;
}

void ringBufferPushBlocking(RingBuffer* r, uint8_t data)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		while (r->len >= RING_BUFFER_SZ)
			;
		ringBufferPushNonBlocking(r, data);
	}
}

uint8_t ringBufferPopNonBlocking(RingBuffer* r)
{
	uint8_t data=0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if (r->len)
		{
			data = r->bufferData[(r->headIdx + RING_BUFFER_SZ - r->len)%RING_BUFFER_SZ];
			r->len--;
		}
	}
	return(data);
}

uint8_t ringBufferPopBlocking(RingBuffer* r)
{
	uint8_t data;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		while (r->len == 0)
			;
		data = ringBufferPopNonBlocking(r);
	}
	return(data);
}

#endif

