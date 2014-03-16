/*************************************************************************
Title:    MRBus Door Terminal
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include <stdbool.h>

#include "mrbus.h"
#include "mrbus_bootloader_builtins.h"

#include "eax_aes.h"

#define printf(format, ...) printf_P (PSTR(format), ##__VA_ARGS__)
#define lprintf(format, ...) fprintf_P (stderr, PSTR(format), ##__VA_ARGS__)

uint8_t mrbus_dev_addr = 0;
uint8_t pkt_count = 0;


//Port B
#define MOSI                  3
#define MISO                  4
#define SCK                   5

//port D
#define NFC_SS                4
#define VFD_RESET__NFC_WAKE   5
#define VFD_SS                6
#define VFD_BUSY              7


#define EE_MASTER_SALT 4


#define MASTERKEY "yourkeygoeshere" //16 bytes
aes128_ctx_t master_aes_ctx;//maybe not used outside of cmacctx. drop as global?
cmac_aes_ctx_t master_cmac_ctx;//maybe not needed on master after testing
uint32_t master_key_salt;


//client definable setting
#define RDP_RECV_OVERLOAD 6
#define RDP_MAX_RECV_USER_PKT_SIZE 100
#define RDP_MAX_TX_OVERLOAD 6
#define RDP_MAX_TX_USER_PKT_SIZE 100



//doesn't change
#define RDP_SEQ_N 16 //size of the packet type allocation for mrbus. 
#define RDP_DATA_TAG_SIZE 4
#define RDP_MAX_CONTROL_PKT_SIZE 100
#if RDP_RECV_OVERLOAD > 8
#error buffer too big. limited by RDP_SEQ_N and bits in recvMarkers
#endif
#if RDP_RECV_OVERLOAD < 1
#error so, i have to ack a packet before you send it?  do tell me about your temporal network.
#endif
#define RDP_MAX_DATA_PER_PKT (MRBUS_BUFFER_SIZE)-6-(RDP_DATA_TAG_SIZE)
#if RDP_MAX_DATA_PER_PKT >= 256
#error buffer too big. coded to need less than a byte for size here
#endif
#define RDP_MAX_RECV_PKT_SIZE max((RDP_MAX_RECV_USER_PKT_SIZE), (RDP_MAX_CONTROL_PKT_SIZE))
#if RDP_MAX_RECV_PKT_SIZE > 0xfff
#error buffer too big. coded to need less than a 12 bits for size here
#endif

typedef struct {
	uint8_t missingPkt[RDP_MAX_DATA_PER_PKT];
	uint16_t dataLen;
	uint8_t pktsInData;
	uint8_t collapsedData[0];
} RDP_buffer_slot_t; 

typedef struct {
	uint16_t channel_and_len;
	uint8_t data[RDP_MAX_RECV_PKT_SIZE];
} RDP_recv_datagram_t; 

typedef struct {
	uint16_t pktNum;
	uint16_t timer;
	uint8_t mrbPkt[MRBUS_BUFFER_SIZE];
} RDP_tx_pkt_queue_t; 

#define RDP_RECV_PKT_BUFFER_SIZE max(RDP_RECV_OVERLOAD*sizeof(RDP_buffer_slot_t), sizeof(RDP_recv_datagram_t)+sizeof(RDP_buffer_slot_t))

#define ct_assert(e) ((void)sizeof(char[1 - 2*!(e)]))

typedef struct {
uint8_t addr;

cmac_aes_ctx_t cmac_ctx;
uint16_t eax_nonce[3];

uint8_t sendOverload;
uint16_t sendPktCounter;
RDP_tx_pkt_queue_t txPktBuffer[RDP_MAX_TX_OVERLOAD];

uint16_t recvPktCounter;
#if RDP_RECV_OVERLOAD > 1
uint16_t recvPktFirstMissing;
uint8_t recvMarkers;
uint8_t recvBuffer[RDP_RECV_PKT_BUFFER_SIZE];
uint8_t recvBufferCount;
#endif
} client_t; 

#define MAXCLIENTS 1
client_t clients[MAXCLIENTS];


#define RING_BUFFER_SZ 20
#define RING_BUFFER_NAME VFD_
#include"../libs/avr-ringbuffer/avr-ringbuffer.h"
#undef RING_BUFFER_SZ
#undef RING_BUFFER_NAME
VFDRingBuffer VFDringBuffer;


// ******** Start 100 Hz Timer - Very Accurate Version

volatile uint8_t ticks;
volatile uint16_t decisecs=0;
volatile uint16_t update_decisecs=10;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCCR1A = 0;
	TCCR1B = _BV(CS11) | _BV(CS10);
	TCCR1C = 0;
	TIMSK1 = _BV(TOIE1);
	ticks = 0;
	decisecs = 0;
}

ISR(TIMER1_OVF_vect)
{
	TCNT1 += 0xF3CB;
	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;
	}
}


ISR(SPI_STC_vect)
{
	if (VFD_ringBufferDepth(&VFDringBuffer) && VFD_timer==0 && (PIND&(1<<VFD_BUSY)))
	{
		SPDR = VFD_ringBufferPop(&VFDringBuffer)
		VFD_timer=;
	}
}

void RDP_USER_DATA_RECV(client_t *c, uint8_t channel, uint16_t len, uint8_t *data)
{
//todo
}


void RDP_DATA_RECV(client_t *c)
{
	while (1)
	{
		if (c->recvBufferCount < 2)
			return;

		RDP_recv_datagram_t *dg=(void*)c->recvBuffer;
		uint16_t len=dg->channel_and_len;
		uint8_t channel=len>>12;
		len=len&0xfff;
		if (c->recvBufferCount < 2+len)
			return;

		uint8_t *data = c->recvBuffer+2;
		if (channel==0)//control	 
		{
		//todo
		}
		else if (channel!=0xf)//not discard
			RDP_USER_DATA_RECV(c, channel, len, data);
			
		memmove(c->recvBuffer, c->recvBuffer+len, RDP_RECV_PKT_BUFFER_SIZE-len);
		c->recvBufferCount-=len;
	}
}

void RDP_ACK(client_t *c, uint16_t pktNum, uint8_t code)
{
//todo: scan outgoing mrbus queue for an ack for this client that I could merge with this one using the "up to" flag
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];
	uint8_t hbuf[6];

	hbuf[1] = txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
	hbuf[0] = txBuffer[MRBUS_PKT_DEST] = c->addr;
	hbuf[2] = txBuffer[MRBUS_PKT_LEN] = 6+3+8;
	hbuf[3] = txBuffer[MRBUS_PKT_TYPE] = 16+1-128;
	*(uint16_t*)(hbuf+4) = *(uint16_t*)(txBuffer+6) = pktNum; 
	txBuffer[8] = code;
	c->eax_nonce[2] = pktNum|0X8000;
	//encrypt
	eax_aes_enc(&c->cmac_ctx, txBuffer+8, 8, (uint8_t*)c->eax_nonce, 6, hbuf, 6, txBuffer+8, txBuffer[MRBUS_PKT_LEN]-8-8);
	mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);

}
void RDP_tx(client_t *c, uint8_t channel, uint16_t len, uint8_t *data)
{

//uint8_t sendOverload;
//uint16_t sendPktCounter;
//RDP_tx_pkt_queue_t txPktBuffer[RDP_MAX_TX_OVERLOAD];

	uint8_t hbuf[4];
	uint8_t *txBuffer = c->txPktBuffer[0].mrbPkt;//ASDF!
	do
	{

		uint8_t *p=txBuffer+MRBUS_PKT_TYPE+1;
		uint8_t pdlen=5;//ASDF!

		uint16_t pktNum = c->sendPktCounter;

		hbuf[1] = txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		hbuf[0] = txBuffer[MRBUS_PKT_DEST] = c->addr;
		hbuf[2] = txBuffer[MRBUS_PKT_LEN] = 6+pdlen+RDP_DATA_TAG_SIZE;
		hbuf[3] = txBuffer[MRBUS_PKT_TYPE] = (pktNum%16)-128;

		c->eax_nonce[2] = pktNum;
		//encrypt
		eax_aes_enc(&c->cmac_ctx, txBuffer+6, RDP_DATA_TAG_SIZE, (uint8_t*)c->eax_nonce, 6, hbuf, 4, txBuffer+6, txBuffer[MRBUS_PKT_LEN]-6-RDP_DATA_TAG_SIZE);
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);

		len-=pdlen;
		data+=pdlen;
	}while(len);
}


void PktHandler(void)
{
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];
	
	if (0 == mrbusPktHandlerStart(&mrbusRxQueue, rxBuffer, sizeof(rxBuffer), txBuffer, sizeof(txBuffer)))
PktIgnore:
		return;
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
	
	if ('W' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)rxBuffer[6], rxBuffer[7]);
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = rxBuffer[7];
		if (MRBUS_EE_DEVICE_ADDR == rxBuffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;	
	}
	else if ('R' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'r';
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rxBuffer[6]);			
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Version
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 18;
		txBuffer[MRBUS_PKT_TYPE] = 'v';
		txBuffer[6]  = 'N';
		txBuffer[7]  = 'f';
		txBuffer[8]  = 'c';
		txBuffer[9]  = 'D';
		txBuffer[10]  = 'o';
		txBuffer[11]  = 'o';
		txBuffer[12]  = 'r';
		txBuffer[13]  = 0; // Software Revision
		txBuffer[14]  = 0; // Software Revision
		txBuffer[15]  = 0; // Software Revision
		txBuffer[16]  = 0; // Hardware Major Revision
		txBuffer[17]  = 0; // Hardware Minor Revision
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('X' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Reset
		cli();
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = _BV(WDE);
		while(1);  // Force a watchdog reset
		sei();
	}
	else if ('Z' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// aes test
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_TYPE] = 'z';
		uint8_t l = min(16, rxBuffer[MRBUS_PKT_LEN]-6);
		uint8_t buf[16];
		for(i=0;i<l;i++)
		  buf[i] = rxBuffer[6+i];
		for(;i<16;i++)
		  buf[i] = 0;
		aes128_enc(buf, &master_aes_ctx);
		l=min(MRBUS_BUFFER_SIZE, 16+6);
		txBuffer[MRBUS_PKT_LEN] = l;
		for(i=6;i<l;i++)
		  txBuffer[i] = buf[i-6];
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('1' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// omac test
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_TYPE] = '2';
		txBuffer[MRBUS_PKT_LEN] = 20;
		uint8_t l = rxBuffer[MRBUS_PKT_LEN]-6;
		uint8_t buf[16];
		cmac_aes(&master_cmac_ctx, buf, rxBuffer+MRBUS_PKT_TYPE+1, l);
		for(i=0;i<14;i++)
		  txBuffer[i+6] = buf[i];
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('3' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// eax test
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_TYPE] = '4';
		uint8_t nl = rxBuffer[6];
		uint8_t hl = (nl>>4)&0xf;
		nl&=0xf;
		uint8_t dl=rxBuffer[MRBUS_PKT_LEN]-7-nl-hl;
		if (dl+3>14)
			dl=14-3;
		eax_aes_enc(&master_cmac_ctx, txBuffer+6, 3, rxBuffer+7, nl, rxBuffer+7+nl, hl, rxBuffer+7+nl+hl, dl);
		txBuffer[MRBUS_PKT_LEN] = 6+dl+3;
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ((int8_t)rxBuffer[MRBUS_PKT_TYPE] == RDP_SEQ_N-128) 
	{
		//secure channel packet ack
	}
	else if ((int8_t)rxBuffer[MRBUS_PKT_TYPE] < RDP_SEQ_N+1-128) 
	{
		//secure channel packet
		client_t *c=clients;
		for (uint8_t i=0;i<MAXCLIENTS; i++, c++)
			if(c->addr==rxBuffer[MRBUS_PKT_SRC])
				break;
		if(c>=clients+MAXCLIENTS)
			goto PktIgnore;
		uint8_t hbuf[6];
		hbuf[0] = rxBuffer[MRBUS_PKT_DEST];
		hbuf[1] = rxBuffer[MRBUS_PKT_SRC];
		hbuf[2] = rxBuffer[MRBUS_PKT_LEN];
		hbuf[3] = rxBuffer[MRBUS_PKT_TYPE];
		if ((int8_t)rxBuffer[MRBUS_PKT_TYPE] == 16+1-128) 
		{//ack
			if (rxBuffer[MRBUS_PKT_LEN] <6+3)//too short?
				goto PktIgnore;
			uint16_t pktNum = *(uint16_t*)(rxBuffer+6); 
			if (pktNum &0x8000)//illegal pkt num?  (leaves room for the ack and data nonces to not re-use each other
				goto PktIgnore;
			*(uint16_t*)(hbuf+4) = pktNum;
			c->eax_nonce[2] = pktNum|0X8000;
			//verify pkt and decrypt
			if(0 == eax_aes_dec(&c->cmac_ctx, rxBuffer+8, 8, (uint8_t*)c->eax_nonce, 6, hbuf, 6, rxBuffer+8, rxBuffer[MRBUS_PKT_LEN]-8))
				goto PktIgnore;
			//ok, ack recvd pktNum, flags in rxBuffer[8].
		}
		else
		{
			int8_t seqNum = rxBuffer[MRBUS_PKT_TYPE]&0xF;
			uint16_t pktNum = ((seqNum-((c->recvPktCounter&(RDP_RECV_OVERLOAD*2-1))-RDP_RECV_OVERLOAD)-1)&(RDP_RECV_OVERLOAD*2-1))+1+c->recvPktCounter-RDP_RECV_OVERLOAD;//some magic here from clientPktCounter and seqNum; 
			if (pktNum &0x8000)//illegal pkt num?  (leaves room for the ack and data nonces to not re-use each other
				goto PktIgnore;
			c->eax_nonce[2] = pktNum;
			//verify pkt and decrypt
			if(0 == eax_aes_dec(&c->cmac_ctx, rxBuffer+6, RDP_DATA_TAG_SIZE, (uint8_t*)c->eax_nonce, 6, hbuf, 4, rxBuffer+6, rxBuffer[MRBUS_PKT_LEN]-6))
				goto PktIgnore;

			uint8_t sz=rxBuffer[MRBUS_PKT_LEN]-6-RDP_DATA_TAG_SIZE;
			//ok, data recvd pktNum, in rxBuffer+6, size sz
			if(c->recvPktCounter<pktNum)
				c->recvPktCounter=pktNum;

			if(pktNum-c->recvPktFirstMissing>=RDP_RECV_OVERLOAD)//shouldn't be able to happen if I got my math right above
			{
				RDP_ACK(c, pktNum, 1);
				goto PktIgnore;
			}

			if(pktNum<c->recvPktFirstMissing || c->recvMarkers&(1<<(pktNum-c->recvPktFirstMissing)))//we don't need this again
			{
				RDP_ACK(c, pktNum, 2);
				goto PktIgnore;
			}
			
			RDP_buffer_slot_t *p = (void*)(c->recvBuffer+c->recvBufferCount); 
			int8_t x = pktNum-c->recvPktFirstMissing;

			if(x==0)
			{
				if (RDP_RECV_PKT_BUFFER_SIZE-c->recvBufferCount <sz)//don't particularly worry about overrun.  there is allocated space for a full datagram plus a new packet.
				{//this should only happen if a previous RDP_DATA_RECV ignores a full datagram, which "shouldn't" happen, so lets not worry about handling it very well.
					RDP_DATA_RECV(c);
					goto PktIgnore;
				}
				do
				{
					c->recvMarkers>>=1;
					++c->recvPktFirstMissing;
				}while (c->recvMarkers&1);
				c->recvBufferCount+=sz+p->dataLen;
			}
			else
			{
				while (x > 0 && (void*)(p+1)<=(void*)(c->recvBuffer+RDP_RECV_PKT_BUFFER_SIZE))
				{
					x--;//count the missing packet in this slot
					x-=p->pktsInData;//count the packets that have been collapsed into the data of this slot
					p=(void*)(((uint8_t*)(p+1))+p->dataLen); //advance p
				}
				if(x!=0 || (void*)(p+1)>(void*)(c->recvBuffer+RDP_RECV_PKT_BUFFER_SIZE)) //there is no room in the buffer for this packet
					goto PktIgnore;

				c->recvMarkers|=(1<<(pktNum-c->recvPktFirstMissing));
				(p-1)->pktsInData++;
				(p-1)->dataLen+=sz;
			}
			RDP_ACK(c, pktNum, 0);
			memcpy(p->missingPkt, rxBuffer+6, sz);
			memmove((uint8_t*)p+sz, p->collapsedData, c->recvBuffer+RDP_RECV_PKT_BUFFER_SIZE-p->collapsedData);
			memset(c->recvBuffer+RDP_RECV_PKT_BUFFER_SIZE-(p->collapsedData-((uint8_t*)p+sz)), 0, p->collapsedData-((uint8_t*)p+sz));

			RDP_DATA_RECV(c);
		}


	
	
	}

	// FIXME:  Insert code here to handle incoming packets specific
	// to the device.

	//*************** END PACKET HANDLER  ***************
}

void keySetup(client_t *c){
//worst key derivation function eva!!!
//replace later
	uint8_t buf[16];

	memset(c, 0, sizeof(client_t));
	aes128_ctx_t client_aes_ctx;

	memset(buf+4, 0, 16-4-1);
	memcpy(buf, &master_key_salt, sizeof(master_key_salt));
	buf[15] = 1;
	aes128_enc(buf, &master_aes_ctx);
	aes128_init(buf, &client_aes_ctx);
	cmac_aes_init(&client_aes_ctx, &c->cmac_ctx);

	memset(buf+4, 0, 16-4-1);
	memcpy(buf, &master_key_salt, sizeof(master_key_salt));
	buf[15] = 2;
	aes128_enc(buf, &master_aes_ctx);
	memcpy(c->eax_nonce, buf, 4);
	
}


void init(void)
{
	//set all pins to input with pull ups..
	MCUCR=0;
	PORTB = PORTC = PORTD = 0xff;
	DDRB = DDRC = DDRD = 0;

	PORTD = (1<<NFC_SS)|(1<<VFD_RESET__NFC_WAKE);

	// Set MOSI, SCK, SSs, and reset to output.  also set PB2, which is SPI SS. needs to be output for master mode to work.  mabe move one of our SSs here later 
	DDRB = (1<<MOSI)|(1<<SCK)|(1<<2);
	DDRD = (1<<NFC_SS)|(1<<VFD_SS)|(1<<VFD_RESET__NFC_WAKE);

	// Enable SPI, Master, set clock rate fck/16, SPI mode 1,1
	PRR &=~(1<<PRSPI);
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<CPOL)|(1<<CPHA);
	
	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif	
	
	_delay_ms(2);
	PORTD &= ~(1<<VFD_RESET__NFC_WAKE);
	_delay_ms(2);
	PORTD |= (1<<VFD_RESET__NFC_WAKE);


	pkt_count = 0;

	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}
	
	update_decisecs = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
		| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);

	// This line assures that update_decisecs is at least 1
	update_decisecs = max(1, update_decisecs);
	
	// FIXME: This line assures that update_decisecs is 2 seconds or less
	// You probably don't want this, but it prevents new developers from wondering
	// why their new node doesn't transmit (uninitialized eeprom will make the update
	// interval 64k decisecs, or about 110 hours)  You'll probably want to make this
	// something more sane for your node type, or remove it entirely.
	update_decisecs = min(20, update_decisecs);


/*
// Uncomment this block to set up the ADC to continuously monitor the bus voltage using a 3:1 divider tied into the ADC7 input
// You also need to uncomment the ADC ISR near the top of the file
	// Setup ADC
	ADMUX  = 0x47;  // AVCC reference; ADC7 input
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC

	busVoltage = 0;
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
*/

	VFD_ringBufferInitialize(&VFDringBuffer);

	FILE log_str = FDEV_SETUP_STREAM(log_putchar, log_getchar, _FDEV_SETUP_RW);
	FILE vfd_str = FDEV_SETUP_STREAM(vfd_putchar, NULL, _FDEV_SETUP_WRITE);

	stdout = stdin = &vfd_str;
	stderr = &log_str;


aes128_init(MASTERKEY, &master_aes_ctx);
cmac_aes_init(&master_aes_ctx, &master_cmac_ctx);
master_key_salt = 0;//uncomment once out of testing this part eeprom_read_dword((const uint32_t*)EE_MASTER_SALT);

for (uint8_t i=0;i<MAXCLIENTS; i++)
	keySetup(clients+i);




}

#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 8

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

int main(void)
{
ct_assert( (RDP_RECV_PKT_BUFFER_SIZE)<256 );//just a test.  if this fails, yo uhave chosen buffer sizes that require you to change client_t.recvBufferCount to a uint16_t. I can't do it automatically since sizeof isn't available to the preprocessor

	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbusInit();

	sei();	

	while (1)
	{
		wdt_reset();

#ifdef MRBEE
		mrbeePoll();
#endif
		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
			PktHandler();
			
		// FIXME: Do any module-specific behaviours here in the loop.
		
		if (decisecs >= update_decisecs && !(mrbusPktQueueFull(&mrbusTxQueue)))
		{
			uint8_t txBuffer[MRBUS_BUFFER_SIZE];

			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 9;
			txBuffer[5] = 'S';
			txBuffer[6] = 'h';
			txBuffer[7] = 'i';
			txBuffer[8] = pkt_count++;
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			decisecs = 0;
		}	

		if (mrbusPktQueueDepth(&mrbusTxQueue))
		{
			uint8_t fail = mrbusTransmit();

			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit to avoid hammering the bus
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			if (fail)
			{

#ifndef MRBEE
				uint8_t bus_countdown = 20;
				while (bus_countdown-- > 0 && !mrbusIsBusIdle())
				{
					wdt_reset();
					_delay_ms(1);
					if (mrbusPktQueueDepth(&mrbusRxQueue))
						PktHandler();
				}
#endif
			}
		}
	}
}



