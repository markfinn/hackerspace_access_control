/*************************************************************************
Title:    AVR Ringbuffer
Authors:  Mark Finn <mark@mfinn.net>, Green Bay, WI, USA
          Nathan Holmes <maverick@drgw.net>, Colorado, USA
File:     avr-ringbuffer.h
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2014 Mark Finn, Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License along 
    with this program. If not, see http://www.gnu.org/licenses/
    
*************************************************************************/

#ifndef RING_BUFFER_SZ
#define _RING_BUFFER_SZ  128
#else
#define _RING_BUFFER_SZ  RING_BUFFER_SZ
#endif

#ifndef RING_BUFFER_NAME
#define _RING_BUFFER_NAME
#else
#define _RING_BUFFER_NAME  RING_BUFFER_NAME
#endif

#ifndef _AVR_RINGBUFFER_H_RING_BUFFER_NAME
#define _AVR_RINGBUFFER_H_RING_BUFFER_NAME

//go crazy with nexted expansions to make the name work right.  Yay CPP tricks.
#define _RING_BUFFER_NAMEIFY__(r, n) r ## n
#define _RING_BUFFER_NAMEIFY_(r, n) _RING_BUFFER_NAMEIFY__(r, n)
#define _RING_BUFFER_NAMEIFY(n) _RING_BUFFER_NAMEIFY_(_RING_BUFFER_NAME, n)

typedef struct
{
	volatile uint8_t headIdx;
	volatile uint8_t len;	
	volatile uint8_t bufferData[_RING_BUFFER_SZ];
} _RING_BUFFER_NAMEIFY(RingBuffer);


void _RING_BUFFER_NAMEIFY(ringBufferInitialize)(_RING_BUFFER_NAMEIFY(RingBuffer)* r)
{
	r->headIdx = r->len = 0;
}


uint8_t _RING_BUFFER_NAMEIFY(ringBufferDepth)(_RING_BUFFER_NAMEIFY(RingBuffer)* r)
{
	return(r->len);
}

uint8_t _RING_BUFFER_NAMEIFY(ringBufferPushNonBlocking)(_RING_BUFFER_NAMEIFY(RingBuffer)* r, uint8_t data)
{
	uint8_t res=0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if (r->len < _RING_BUFFER_SZ)
		{
			r->bufferData[r->headIdx++] = data;
			r->len++;
			if( r->headIdx >= _RING_BUFFER_SZ )
				r->headIdx = 0;
			res=1;
		}
	}
	return(res);
}

void _RING_BUFFER_NAMEIFY(ringBufferPushBlocking)(_RING_BUFFER_NAMEIFY(RingBuffer)* r, uint8_t data)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		while (r->len >= _RING_BUFFER_SZ);

		_RING_BUFFER_NAMEIFY(ringBufferPushNonBlocking)(r, data);
	}
}

uint8_t _RING_BUFFER_NAMEIFY(ringBufferPopNonBlocking)(_RING_BUFFER_NAMEIFY(RingBuffer)* r)
{
	uint8_t data=0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if (r->len)
		{
			data = r->bufferData[(r->headIdx + _RING_BUFFER_SZ - r->len) % _RING_BUFFER_SZ];
			r->len--;
		}
	}
	return(data);
}

uint8_t _RING_BUFFER_NAMEIFY(ringBufferPopBlocking)(_RING_BUFFER_NAMEIFY(RingBuffer)* r)
{
	uint8_t data;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		while (r->len == 0);

		data = _RING_BUFFER_NAMEIFY(ringBufferPopNonBlocking)(r);
	}
	return(data);
}


#endif

#undef _RING_BUFFER_SZ
#undef _RING_BUFFER_NAME

