//don't like 'X' command, but out of space
//vectors:  aes, cbcmac, directtoloader(loaderactivate=1 and run somehow.)  mrbus stuff?

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
#include <util/delay.h>

#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>

#include "mrbus.h"

#include "aes_types.h"
#include "aes128_enc.h"
#include "aes_keyschedule.h"

#include <avr/signature.h>

#define BOOTLOADERVER 1

#define LOADERPKTS ((SPM_PAGESIZE+11)/12)
#define LOADERSTATBYTES ((LOADERPKTS+7)/8)
#if LOADERPKTS > 256
#error LOADERPKTS > 256
#endif
uint8_t loaderstatus[LOADERSTATBYTES];
uint8_t loaderactivate=0;
int16_t loaderpage=-1;
uint8_t mrbus_dev_addr = 0;

uint8_t pkt_count = 0;



uint32_t getsz()
{
  //find the signature, as the 20 bytes just prior to the start (but after the interupt vector)
  //[reset vect]
  //[other vectors...]
  //[prog size (4 bytes)]
	//reset
	//prog
	//sig
	  
	//size includes all bytes but sig
	uint16_t reset = *(uint16_t*)0;

  if (reset>>8 != 0xC0) //check for short rjmp 
    return 0;

  uint8_t *start=(uint8_t*)(((uint16_t*)(void*)0)+((reset&0xff)+1));
  uint8_t sz=*(((uint32_t*)start)-1);
  if (sz < 2 || sz+4 > FLASHEND) //check for bad values 
    return 0;
  return sz;
}

uint8_t* getsigptr()
{
  uint8_t *sig=((uint8_t*)0)+getsz();
  return sig;
}

void lenpadcbcmacaes(uint8_t *data, uint8_t *key, uint8_t *m, uint32_t sz)
{
	aes128_ctx_t ctx; /* the context where the round keys are stored */
	aes128_init(key, &ctx); /* generating the round keys from the 128 bit key */

	memset(data, 0, 16);
  *(uint32_t*)data=sz;

	aes128_enc(data, &ctx);
  while (sz)
  {
  	for(int i=0;i<16 && i<sz;i++)
			data[i]^=m[i];
		m += 16;
		sz -= 16;
		aes128_enc(data, &ctx);
  }
}

uint8_t sigcheck()
{
	uint8_t out[16];
	uint8_t key[]  = { 0x01, 0x23, 0x45, 0x67,
                   0x89, 0xAB, 0xCD, 0xEF,
                   0x01, 0x23, 0x45, 0x67,
                   0x89, 0xAB, 0xCD, 0xEF };

	uint32_t sz = getsz();
  lenpadcbcmacaes(out, key, (uint8_t*)0, sz);
	uint8_t* sig = getsigptr();
  for(int i=0; i<16; i++, sig++)
    if (out[i] != *sig)
      return 1;
  while (sig<=((uint8_t*)0)+FLASHEND)
    if (*(sig++)!=0xff)
      return 1;
  return 0;
}



void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	if (0 == mrbusPktQueuePop(&mrbusRxQueue, rxBuffer, sizeof(rxBuffer)))
		return;


	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		return;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		return;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
		return;
		
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
		goto shortreturnsend;
	}
	if ('!' == rxBuffer[MRBUS_PKT_TYPE])
	{
		// BOOT LOADER STATUS packet
		loaderactivate=1;
statussend:
		txBuffer[MRBUS_PKT_LEN] = 6+3;
		txBuffer[MRBUS_PKT_TYPE] = '@';
		for(i=0; i<LOADERSTATBYTES-1; i++)
	  	if(loaderstatus[i]!=0xff)
	  		break;
		txBuffer[6] = i;
		txBuffer[7] = loaderstatus[i];
		txBuffer[8] = loaderpage>0;

returnsend:
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		return;	
	} 
	else if ('D' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// DATA
		if (rxBuffer[MRBUS_PKT_LEN]!= 20)
			return;	
		uint8_t x = rxBuffer[18];
 		if(loaderpage<0 || x >= LOADERPKTS || (loaderstatus[x/8]&(1<<(x&7))))
			return;	
 		uint16_t addr = loaderpage + x*12;
		uint16_t *p = (uint16_t *)(rxBuffer+6);
		cli();
		for (i=0; i<6 && addr < SPM_PAGESIZE; i++, addr+=2)
			boot_page_fill (addr, *(p++));
		sei();
		loaderstatus[x/8]|=(1<<(x&7));
		if(rxBuffer[19])
			goto statussend;
	}
	else if ('E' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// ERASE PAGE
		if (rxBuffer[MRBUS_PKT_LEN]!= 8)
			return;	
		//blank statuses
		for(i=0; i<LOADERSTATBYTES; i++)
	  	loaderstatus[i]=0;
		loaderpage = *(uint16_t *)(rxBuffer+6);
		int x=loaderpage*SPM_PAGESIZE;
		if (loaderpage<0 || x+SPM_PAGESIZE > BOOTSTART)
		{
			loaderpage = -1;
			return;
		}
		cli();
		boot_rww_enable ();//clears page buffer, strangely.
		boot_page_erase (loaderpage);
		sei();
		boot_spm_busy_wait (); 
		
		txBuffer[MRBUS_PKT_TYPE] = 'e';
shortreturnsend:
		txBuffer[MRBUS_PKT_LEN] = 6;
		goto returnsend;
	}
	else if ('W' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// WRITE PAGE
		if (rxBuffer[MRBUS_PKT_LEN]!= 6)
			return;	
		cli();
		boot_page_write (loaderpage);     // Store buffer in flash page.
		loaderpage=-1;
		sei();
		boot_spm_busy_wait (); 
		cli();
		boot_rww_enable ();
		sei();
		txBuffer[MRBUS_PKT_TYPE] = 'w';
		goto shortreturnsend;
	}
	else if ('S' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Signature
		txBuffer[MRBUS_PKT_LEN] = 20;
		txBuffer[MRBUS_PKT_TYPE] = 's';
		txBuffer[6]  = '!';
		txBuffer[7]  = sigcheck();
		uint8_t* p=getsigptr();
		for(i=8; i<20; i++, p++)
			txBuffer[i]  = *p;
		goto returnsend;
	}
	else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Version
		txBuffer[MRBUS_PKT_LEN] = 19;
		txBuffer[MRBUS_PKT_TYPE] = 'v';
		txBuffer[6]  = '!';
		txBuffer[7]  = BOOTLOADERVER;
		txBuffer[8]  = SIGNATURE_0;
		txBuffer[9]  = SIGNATURE_1;
		txBuffer[10]  = SIGNATURE_2;
		txBuffer[11]  = SPM_PAGESIZE>>8;
		txBuffer[12]  = SPM_PAGESIZE;
		txBuffer[13]  = FLASHEND>>16;
		txBuffer[14]  = FLASHEND>>8;
		txBuffer[15]  = FLASHEND;
		txBuffer[16]  = BOOTSTART>>16;
		txBuffer[17]  = BOOTSTART>>8;
		txBuffer[18]  = BOOTSTART;

		goto returnsend;
	}
	else if ('X' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		cli();
		asm("jmp 0000");
/*		// Reset
		cli();
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = _BV(WDE);
		while(1);  // Force a watchdog reset
*/	}

}

void init(void)
{
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

	pkt_count = 0;

	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
	}
	
}

#define MRBUS_TX_BUFFER_DEPTH 8
#define MRBUS_RX_BUFFER_DEPTH 24

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

int main(void)
{
	// Application initialization
	init();
	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbusInit();
	sei();	

	//send an "I'm here!" broadcast to help in catching the bootloader
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];
	txBuffer[MRBUS_PKT_LEN] = 6;
	txBuffer[MRBUS_PKT_TYPE] = '@';
	txBuffer[MRBUS_PKT_DEST] = 0xff;
	txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
	mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);


	uint8_t bus_countdown = 100;
	while (bus_countdown-- > 0 && !loaderactivate)
	{
		wdt_reset();
		_delay_ms(10);
		if (mrbusPktQueueDepth(&mrbusRxQueue))
		PktHandler();
	}

	if(!loaderactivate && !sigcheck())
	{
		cli();
		// Put interrupts back in app land
		MCUCR = (1<<IVCE);
		MCUCR = 0;
		asm("jmp 0000");
	}

	while (1)
	{
		wdt_reset();

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
			PktHandler();
			
		if (mrbusPktQueueDepth(&mrbusTxQueue))
			mrbusTransmit();
	}
}


