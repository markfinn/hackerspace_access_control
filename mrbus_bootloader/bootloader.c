/*************************************************************************
Title:    MRBus bootloader
Authors:  Mark Finn <mark@mfinn.net>
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2014 Mark Finn

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>

#include "mrbus.h"

#include "aes_types.h"
#include "aes128_enc.h"
#include "aes_keyschedule.h"

#include <avr/signature.h>


#define initpins PORTB = PORTC = PORTD = 0xff;


#define BOOTLOADERVER 2

#define LOADERPKTS ((SPM_PAGESIZE+11)/12)
#define LOADERSTATBYTES ((LOADERPKTS+7)/8)
#if LOADERPKTS > 256
#error LOADERPKTS > 256
#endif

uint8_t loaderstatus[LOADERSTATBYTES];
uint8_t loaderactivate=0;
uint8_t bus_countdown = 100;
uint8_t mrbus_dev_addr;
uint8_t sigbuf[16];

uint8_t progbuf[SPM_PAGESIZE];

void debug(uint8_t len, uint8_t *bytes)
{
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];
	txBuffer[MRBUS_PKT_DEST] = 0xff;
	txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
	txBuffer[MRBUS_PKT_TYPE] = '*';
	if (len > 14)
		len=14;
	txBuffer[MRBUS_PKT_LEN] = 6+len;
	memcpy(txBuffer+6, bytes, len);
	mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
}

uint8_t  getbootloaderver()
{//a function so that the application can ask too.
	return BOOTLOADERVER;
}

void  __attribute__ ((section (".text_pageload"))) boot_program_page  (uint32_t page, uint8_t *buf) 
{//from http://www.nongnu.org/avr-libc/user-manual/group__avr__boot.html#ga7249d12e06789cd306583abf7def8176
	uint16_t i;
	uint8_t sreg;

	// Disable interrupts.
	sreg = SREG;

	cli();
	eeprom_busy_wait ();
	boot_page_erase (page);
	boot_spm_busy_wait ();      // Wait until the memory is erased.

	for (i=0; i<SPM_PAGESIZE; i+=2)
	{
		// Set up little-endian word.
		uint16_t w = *buf++;
		w += (*buf++) << 8;
		boot_page_fill (page + i, w);
	}

	boot_page_write (page);     // Store buffer in flash page.
	boot_spm_busy_wait();       // Wait until the memory is written.

	// Reenable RWW-section again. We need this if we want to jump back
	// to the application after bootloading.
	boot_rww_enable ();

	// Re-enable interrupts (if they were ever enabled).

	SREG = sreg;
}

uint16_t getsz()
{
	uint16_t s = pgm_read_word(BOOTSTART-2);
	if (s>BOOTSTART-2-16)
		return 0;
	return s;
}

void loadsig()
{
	memcpy_P(sigbuf, (PGM_P)(BOOTSTART-2-16), 16);
}

uint8_t pgmreadbyte(uint16_t addr)
{
	return pgm_read_byte(addr);
}

void  lenpadcbcmacaes(uint8_t *data, uint8_t *key, uint16_t sz, uint8_t (*dataget)(uint16_t), uint16_t offset)
{
	aes128_ctx_t ctx; /* the context where the round keys are stored */
	aes128_init(key, &ctx); /* generating the round keys from the 128 bit key */

	memset(data, 0, 16);
	*(uint16_t*)data=sz;

	aes128_enc(data, &ctx);
//debug(4, (uint8_t[]){(sz>>0)&0xff,(sz>>8)&0xff,(sz>>16)&0xff,(sz>>24)&0xff});
	while (sz)
	{
		for(int i=0;i<16 && sz;i++, sz--)
			data[i]^=dataget(i+offset);
		offset += 16;
		aes128_enc(data, &ctx);
	}
}

uint8_t ffcheck(uint16_t sz)
{
	for (uint16_t p = sz;p<BOOTSTART-2-16;p++)
		if (pgm_read_byte(p)!=0xff)
			return 1;
	return 0;
}

uint8_t sigcheck()
{
	uint8_t out[16];
	uint8_t key[16];
//	extern PGM_P user_key; not working.. linker can't find symbol?
//	memcpy_P(key, user_key, 16);
	memcpy_P(key, (PGM_P)0x7054, 16);

	uint16_t sz = getsz();
	loadsig();
	uint8_t* sig = sigbuf;

	if (ffcheck(sz))
		return 1;

	lenpadcbcmacaes(out, key, sz, &pgmreadbyte, 0);
	for(int i=0; i<16; i++, sig++)
		if (out[i] != *sig)
			return 1;
	return 0;
}



	

#define MRBUS_TX_BUFFER_DEPTH 8
#define MRBUS_RX_BUFFER_DEPTH 24

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

int main(void)
{

	initpins;


	// Application initialization
	MCUCR = (1<<IVCE);
	MCUCR = (1<<IVSEL);
#if 1
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif	


	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
	}

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbusInit();
	sei();	

	//send an "I'm here!" broadcast to help in catching the bootloader
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];
	rxBuffer[MRBUS_PKT_SRC]=0xff;
	goto statussend;

	while (1)
	{
		wdt_reset();
		if (mrbusPktQueueDepth(&mrbusRxQueue))
		do
		{
			uint16_t crc = 0;
			uint8_t i;

			if (0 == mrbusPktQueuePop(&mrbusRxQueue, rxBuffer, sizeof(rxBuffer)))
				break;


			//*************** PACKET FILTER ***************
			// Loopback Test - did we send it?  If so, we probably want to ignore it
			if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
				break;

			// Destination Test - is this for us or broadcast?  If not, ignore
			if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
				break;
	
			// CRC16 Test - is the packet intact?
			for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
			{
				if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
					crc = mrbusCRC16Update(crc, rxBuffer[i]);
			}
			if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
				break;
		
			//*************** END PACKET FILTER ***************


			//*************** PACKET HANDLER - PROCESS HERE ***************

			// Just smash the transmit buffer if we happen to see a packet directed to us
			// that requires an immediate response
			//
			// If we're in here, then either we're transmitting, then we can't be 
			// receiving from someone else, or we failed to transmit whatever we were sending
			// and we're waiting to try again.  Either way, we're not going to corrupt an
			// in-progress transmission.
			//
			// All other non-immediate transmissions (such as scheduled status updates)
			// should be sent out of the main loop so that they don't step on things in
			// the transmit buffer
	
			if ('A' == rxBuffer[MRBUS_PKT_TYPE])
			{
				// PING packet
				txBuffer[MRBUS_PKT_TYPE] = 'a';
		shortreturnsend:
				txBuffer[MRBUS_PKT_LEN] = 6;
				goto returnsend;
			}
			if ('!' == rxBuffer[MRBUS_PKT_TYPE])
			{
				// BOOT LOADER STATUS packet
				loaderactivate=1;
		statussend:
				txBuffer[MRBUS_PKT_LEN] = 8;
				txBuffer[MRBUS_PKT_TYPE] = '@';
				for(i=0; i<LOADERSTATBYTES-1; i++)
					if(loaderstatus[i]!=0xff)
						break;
				txBuffer[6] = i;
				txBuffer[7] = loaderstatus[i];

		returnsend:
				txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
				txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
				mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
				break;	
			} 
			// Destination Test - is this for us ?   don't respond to these other commands on broadcast
			if (mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
				break;
			
			if ('D' == rxBuffer[MRBUS_PKT_TYPE]) 
			{
				// DATA
				if (rxBuffer[MRBUS_PKT_LEN]!= 20)
					break;	
				uint8_t x = rxBuffer[18];
		 		if(x >= LOADERPKTS)
					break;	
				memcpy(progbuf+x*12, rxBuffer+6, min(12, SPM_PAGESIZE - x*12));
				loaderstatus[x/8]|=(1<<(x&7));
				if(rxBuffer[19])
					goto statussend;
			}
			else if ('F' == rxBuffer[MRBUS_PKT_TYPE]) 
			{
				// FILL PAGE BUF

					uint16_t start;
					uint8_t len, dest;
					
				// ERASE buf markers
				for(i=0; i<LOADERSTATBYTES; i++)
			  	loaderstatus[i]=0;
				txBuffer[MRBUS_PKT_TYPE] = 'f';
				if (rxBuffer[MRBUS_PKT_LEN]== 7)
				{
					memset(progbuf, rxBuffer[6], SPM_PAGESIZE);
					goto shortreturnsend;
				}
				else if (rxBuffer[MRBUS_PKT_LEN]== 10)
				{
					start= *(uint16_t *)(rxBuffer+6);
					len=rxBuffer[8];
					dest=rxBuffer[9];
				}
				else
					break;	
				//todo shorten len if needed, then copy
				if (dest > SPM_PAGESIZE)
					dest = SPM_PAGESIZE;
				if (len > SPM_PAGESIZE - dest)
					len = SPM_PAGESIZE - dest;
				if (len > BOOTSTART-18 - start)//don't allow a copy to a place where we can read back (as length or sig).  that would allow arbitrary reading.
					len = BOOTSTART-18 - start;
				memcpy_P(progbuf+dest, (PGM_P)start, len);

				goto shortreturnsend;
			}
			else if ('#' == rxBuffer[MRBUS_PKT_TYPE]) 
			{
				// WRITE PAGE
				if (rxBuffer[MRBUS_PKT_LEN]!= 8)
					break;	
				uint16_t page = *(uint16_t *)(rxBuffer+6);
				boot_program_page (page, progbuf);
				// ERASE buf markers
				for(i=0; i<LOADERSTATBYTES; i++)
			  	loaderstatus[i]=0;
				txBuffer[MRBUS_PKT_TYPE] = '$';
				goto shortreturnsend;
			}
			else if ('S' == rxBuffer[MRBUS_PKT_TYPE]) 
			{
				// Signature
				txBuffer[MRBUS_PKT_LEN] = 17;
				txBuffer[MRBUS_PKT_TYPE] = 's';
				uint16_t sz = getsz();
				txBuffer[6]  = (sigcheck()?0x80:0)|(ffcheck(sz)?0x01:0);//could be abused to check an arbitrary byte for FF, but seems ok enough.
				txBuffer[7]  = sz;
				txBuffer[8]  = sz>>8;
				//called by sigcheck: loadsig();
				uint8_t* p=sigbuf;
				for(i=9; i<9+8; i++, p++)
					txBuffer[i]  = *p;

				goto returnsend;
			}
			else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
			{
				// Version
				txBuffer[MRBUS_PKT_LEN] = 15;
				txBuffer[MRBUS_PKT_TYPE] = 'v';
				txBuffer[6]  = '!';
				txBuffer[7]  = getbootloaderver();
				*(uint16_t*)(txBuffer+8)  = SPM_PAGESIZE;
				*(uint16_t*)(txBuffer+10)  = BOOTSTART;
				txBuffer[12]  = SIGNATURE_0;
				txBuffer[13]  = SIGNATURE_1;
				txBuffer[14]  = SIGNATURE_2;

				goto returnsend;
			}
			else if ('X' == rxBuffer[MRBUS_PKT_TYPE]) 
			{
				if (rxBuffer[MRBUS_PKT_LEN]== 7 && rxBuffer[6])
				{
					loaderactivate=0;
					bus_countdown=0;
				}
				else
				{
					// Reset
					cli();
					wdt_reset();
					MCUSR &= ~(_BV(WDRF));
					WDTCSR |= _BV(WDE) | _BV(WDCE);
					WDTCSR = _BV(WDE);
					while(1);  // Force a watchdog reset
				}
			}

		}while(0);
		else if (mrbusPktQueueDepth(&mrbusTxQueue))
			mrbusTransmit();
		else if (bus_countdown)
		{
			bus_countdown--;
			_delay_ms(10);
		}
		else if(!loaderactivate && !sigcheck())
		{
			cli();
			// Put interrupts back in app land
			MCUCR = (1<<IVCE);
			MCUCR = 0;
			asm("jmp 0000");
		}

	}

}




