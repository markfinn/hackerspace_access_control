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

#define DEBUGPRINT VFD //undefined, defined, or VFD


#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <avr/pgmspace.h>

#include "mrbus.h"
#include "mrbus_bootloader_builtins.h"

#include "eax_aes.h"
#include "vfd.h"


typedef struct {
	uint8_t len;
	uint32_t num;
} PINEntry; 
#include "userpins.h"
/*
PINEntry const PINS[] = {
/*00*/{4,1234},
/*01*/{0,12343},
/*02*/{9,999999999}
};
*/

#define LOCK_OPEN_TIME 100

uint8_t mrbus_dev_addr = 0;

 PGM_P fatalError = 0;
 
 //Port B
 #define NFC_SS                0
 #define VFD_RESET__NFC_WAKE   1
 #define SPKR_OC1B             2
 #define MOSI                  3
 #define MISO                  4
 #define SCK                   5
 
 //port C
 //keypad rows on PC2-PC5
 #define KB_ROWS_N             4
 #define KB_ROWS_SHIFT         2
 #define KB_ROWS_PIN          PINC


 //port D
 #define KB_COLS_N             3
 #define KB_COLS              (const char[KB_COLS_N]){3, 4, 5}
 #define KB_COLS_DDR          DDRD
 #define KB_COLS_PORT         PORTD
 #define VFD_SS                6
 #define VFD_BUSY              7 //also drives a pin change int23

const char const KEYMAP[KB_COLS_N * KB_ROWS_N]  = {'#', '9', '6', '3', '0', '8', '5', '2', '*', '7', '4', '1'};
#define KEYS_RESET 0x121 //*-8-#   bitmap: 0001 0010 0001

#define MRBUS_TX_BUFFER_DEPTH 1
#define MRBUS_RX_BUFFER_DEPTH 3

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];


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
#define RDP_TIMEOUT 1
#define RDP_DATA_TAG_SIZE 5 // see RDP_MAX_SIG_FAILS
#define RDP_MAX_SIG_FAILS 4 //up to 8 fails (this number per side) at a tag size of 40 bits gives 21.7 years average to slip a packet in undetected at 200 packets per second bus saturation
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
#if RDP_MAX_TX_OVERLOAD > 8
#error buffer too big. limited by assuming bita in a byte in c->acks and nacks
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
	uint32_t pktNum;
	uint16_t timer;
	uint8_t mrbPkt[MRBUS_BUFFER_SIZE];
} RDP_tx_pkt_queue_t; 

#define RDP_RECV_PKT_BUFFER_SIZE max(RDP_RECV_OVERLOAD*sizeof(RDP_buffer_slot_t), sizeof(RDP_recv_datagram_t)+sizeof(RDP_buffer_slot_t))

#define ct_assert(e) ((void)sizeof(char[1 - 2*!(e)]))

typedef enum {RDP_SYN_RCVD, RDP_OPEN} RDP_STATE_t;
typedef enum {RDP_PKT_SYN=0, RDP_PKT_SYNACK, RDP_PKT_ACK, RDP_PKT_RST, RDP_PKT_DATA} RDP_PACKET_t;




typedef struct {
uint8_t addr;
RDP_STATE_t state : 4;
uint8_t maxpktsize;

cmac_aes_ctx_t cmac_ctx;
uint16_t eax_nonce[3];

uint32_t minack;
uint8_t acks;
uint8_t nacks;


uint8_t sendOverload;
uint16_t sendPktCounter;
RDP_tx_pkt_queue_t txPktBuffer[RDP_MAX_TX_OVERLOAD];

uint8_t failedPkts;
uint16_t recvPktCounter;
#if RDP_RECV_OVERLOAD > 1
uint16_t recvLastPkt;
uint8_t recvMarkers;
uint8_t recvBuffer[RDP_RECV_PKT_BUFFER_SIZE];
uint8_t recvBufferCount;
#endif
uint8_t timer;
} client_t; 

#define MAXCLIENTS 1
client_t clients[MAXCLIENTS];



uint8_t RDPvsQueue=0;



#define RING_BUFFER_SZ 8
#define RING_BUFFER_NAME KBD_
#include"../libs/avr-ringbuffer/avr-ringbuffer.h"
#undef RING_BUFFER_SZ
#undef RING_BUFFER_NAME
KBD_RingBuffer KBD_ringBuffer;

typedef struct {
	uint8_t timeLen;
	uint16_t period;
} ToneEntry; 

ToneEntry const * tonePointer=NULL;
uint8_t toneTimer;



typedef enum {SPI_VFD, SPI_NFC, SPI_NFC_DISCARD} SPIsent_t;
typedef enum {VFD_NEEDSTARTTIMEOUT, VFD_BUSYWAIT, VFD_OK, VFD_SEEMSSTUCK} VFDstate_t;
typedef enum {NFC_POLL, NFC_WRITE, NFC_READ} NFCstate_t;

typedef struct {
SPIsent_t SPI_sent : 2;
VFDstate_t VFD_state : 2;
NFCstate_t NFC_state : 2;
uint8_t VFD_timer;
} SPIStates_t;
SPIStates_t SPIState;
#define RING_BUFFER_SZ 16
#define RING_BUFFER_NAME VFD_
#include"../libs/avr-ringbuffer/avr-ringbuffer.h"
#undef RING_BUFFER_SZ
#undef RING_BUFFER_NAME
VFD_RingBuffer VFD_ringBuffer;


volatile uint16_t deciSecs;
volatile uint16_t ticks50khz;

void reset(void)
{
		cli();
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = _BV(WDE);
		while(1);  // Force a watchdog reset
}

void initialize50kHzTimer(void)
{
	// Set up timer 0 for 50kHz (20uS) interrupts
	TCNT0 = 0;
	OCR0A = 50;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS01);// div/8 prescaler
	TIMSK0 |= _BV(OCIE0A);
}

uint8_t vfdtrysend()
{
	if((SPIState.VFD_state == VFD_BUSYWAIT || SPIState.VFD_state == VFD_SEEMSSTUCK) && SPIState.VFD_timer == 0 && 0==(PIND&(1<<VFD_BUSY)))
		SPIState.VFD_state = VFD_OK;

	if (SPIState.VFD_state == VFD_OK && VFD_ringBufferDepth(&VFD_ringBuffer))
	{
		PORTB |= (1<<NFC_SS);
		PORTD |= (1<<VFD_SS);
		SPCR |= (1<<DORD);
		SPIState.VFD_state=VFD_NEEDSTARTTIMEOUT;
		SPCR |= (1<<SPIE);
		SPDR = VFD_ringBufferPopNonBlocking(&VFD_ringBuffer);
		return 1;
	}
	return 0;
}

ToneEntry const TONE_KEYPRESS[] = {{2,2000},{0,0}};
ToneEntry const TONE_OPEN[] = {{20,2000},{20,1600},{20,1400},{0,0}};
ToneEntry const TONE_FAIL[] = {{20,2200},{2,10},{20,2200},{0,0}};
ToneEntry const TONE_TIMEOUT[] = {{2,3000},{0,0}};

void set_tone_pattern(ToneEntry const * const t)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
	tonePointer=t;
  OCR1A = tonePointer->period;
	toneTimer = tonePointer->timeLen;
	}
}
void tone_generate(){
	if (NULL == tonePointer)
    return;
  if (toneTimer--)
		return;
	tonePointer++;
	if (0==tonePointer->timeLen)
	{
		OCR1A = 10;//make it too high to hear. keeps pin centered
		tonePointer=NULL;
		return;
	}
  OCR1A = tonePointer->period;
	toneTimer = tonePointer->timeLen;
}

void keycodeInsert(unsigned char scancode)
{
	set_tone_pattern(TONE_KEYPRESS);
	KBD_ringBufferPushNonBlocking(&KBD_ringBuffer, KEYMAP[scancode]);
}
void keyboard_scan()
{
	static uint8_t kbd_state=0;
	static uint16_t KEYBDSTATE=0;
	static uint16_t KEYBDPREVSTATE=0;
	static uint8_t KEYBDREPTSTATE=0xff;//nothing pressed, no current key

	if((kbd_state&1)==0)
	{
		#if KB_COLS_N != 3
		#error fix keyboard bits
		#endif
		KB_COLS_PORT=(KB_COLS_PORT|(1<<KB_COLS[0])|(1<<KB_COLS[1])|(1<<KB_COLS[2]))&~(1<<KB_COLS[kbd_state/2]);
		KB_COLS_DDR =(KB_COLS_DDR&~((1<<KB_COLS[0])|(1<<KB_COLS[1])|(1<<KB_COLS[2])))|(1<<KB_COLS[kbd_state/2]);
	}
	else
	{
		KEYBDSTATE<<=KB_ROWS_N;
		KEYBDSTATE |= (KB_ROWS_PIN>>KB_ROWS_SHIFT)&((1<<KB_ROWS_N)-1);
	}
	kbd_state++;
	if(kbd_state>=KB_COLS_N*2)
	{
		kbd_state=0;
		KEYBDSTATE^=(1<<(KB_COLS_N*KB_ROWS_N))-1;

		if ((KEYS_RESET&KEYBDSTATE)==KEYS_RESET)//reset on magic compo
			reset();
			
		uint16_t delta=KEYBDPREVSTATE^KEYBDSTATE;

		if (delta&KEYBDSTATE)
		{//new key pressed
			while(delta&KEYBDSTATE)
			{
				KEYBDREPTSTATE=__builtin_ctz(delta&KEYBDSTATE);
				delta^=1<<KEYBDREPTSTATE;
				keycodeInsert(KEYBDREPTSTATE);
			}
		}
		else if (KEYBDREPTSTATE!=0xff && delta&(1<<(KEYBDREPTSTATE&0xf)))
		{//old repeat key unpressed
			KEYBDREPTSTATE=0xff;
		}
		else if (KEYBDREPTSTATE!=0xff)
		{//advancing repeat timers
			uint8_t t=1+(KEYBDREPTSTATE>>4);
			if(t==11 || t==11+3)
			{//start or continue repeat
				keycodeInsert(KEYBDREPTSTATE&0xf);
				if(t==11+3)
				{//continue repeat
					t=10;
				}
			}
			KEYBDREPTSTATE=(t<<4)|(KEYBDREPTSTATE&0xf);
		}

		KEYBDPREVSTATE=KEYBDSTATE;
		KEYBDSTATE=0;
	}

}


ISR(TIMER0_COMPA_vect)
{
	static uint16_t next=0;
	static uint8_t centiSecs=0;

	ticks50khz++;
	if(ticks50khz==next)
	{
		centiSecs++;
		next+=500;
		if(centiSecs==10)
		{
			centiSecs=0;
			deciSecs++;

		}
		keyboard_scan();
    tone_generate();
	}
	if (SPIState.VFD_timer)
	{
		--SPIState.VFD_timer;
		if(0==(SPCR & (1<<SPIE)))
			vfdtrysend();
	}		
}

void marktime(client_t *c)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		c->timer=deciSecs;
	}
}
uint8_t elapsedtime(client_t *c)
{
	uint16_t t;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		t=deciSecs;
	}
	return t-c->timer;
}

ISR(PCINT2_vect)
{
	if(0==(SPCR & (1<<SPIE)))
		vfdtrysend();
}


ISR(SPI_STC_vect)
{
	if(SPIState.VFD_state == VFD_NEEDSTARTTIMEOUT)//the lst thing was a VFD byte, and we haven't waited for the busy line delay yet
	{
		PORTD &= ~(1<<VFD_SS);
		SPIState.VFD_timer=3;//should only need 1 here, but between jitterm rounding, and sketchy VFD or busy handling...
		SPIState.VFD_state = VFD_BUSYWAIT;
	}

//	if(SPIState.SPI_sent == SPI_NFC)
//		NFC_ringBufferPush(&NFCringBuffer, SPDR)

	if (vfdtrysend())
	{}//uses SPI internally
/*	else if (NFC_ringBufferDepth(&NFCringBuffer))
	{
		PORTD &= ~(1<<VFD_SS);
		PORTB &= ~(1<<NFC_SS);
		SPCR &= ~(1<<DORD);
		SPDR = NFC_ringBufferPop(&NFCringBuffer)
		SPIState.SPI_sent = SPI_NFC;
		SPCR |= (1<<SPIE);
	}*/
	else
		SPCR &= ~(1<<SPIE);
}

static int log_putchar(char c, FILE *stream)
{
  //fill in once streams are working
	return 0;
}
static int log_getchar(FILE *stream)
{
  //fill in once streams are working
	return 0;
}
static int vfd_putchar(char c, FILE *stream)
{
	if (c==0x0a)//lf into crlf
		vfd_putchar(0x0d, stream);
	uint16_t start, elapsed;
	uint8_t res=0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		start = ticks50khz;
	}
	do
	{
		res=VFD_ringBufferPushNonBlocking(&VFD_ringBuffer, c);
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
		elapsed = ticks50khz - start;
		}
	}while(res==0 && elapsed < 4000);
	if(res==0)
	{
		SPIState.VFD_state = VFD_SEEMSSTUCK;
	}
	else
	{
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			if(0==(SPCR & (1<<SPIE)))
					vfdtrysend();
		}
	}
	return 0;
}

static FILE log_str = FDEV_SETUP_STREAM(log_putchar, log_getchar, _FDEV_SETUP_RW);
static FILE vfd_str = FDEV_SETUP_STREAM(vfd_putchar, NULL, _FDEV_SETUP_WRITE);

void nukesessionkey()
{
//set the key to unusable, start negotiation of a new one if it isn't already in progress

}


/*
agree on next master nonce int
the one that wants the larger nonce proves 
- send cmac(cmac(aes(nonce(int,0), int)
- sender invalidates that int for future use
- starts timer to disallow more than 1 start per sec average (5 burst?)
if ok, other one sends proof
- send cmac(aes(nonce(int,0), int)
- sender invalidates that int for future use
if ok, each uses aes(nonce(int,1) and aes(nonce(int,2)
*/

uint16_t masterSaltGetNext(uint16_t salt, uint8_t remote_addr)
{
	//zero is bad
	if (salt==0)
	{
		//other end tried to use an invalid salt.
		clprintE("remote %d attempted 0 master salt", remote_addr);
		return 0;
	}
	uint16_t s = 1+eeprom_read_word((const uint16_t*)EE_MASTER_SALT);
	if (s!=0)
	{
		salt = max(salt, s);

		eeprom_write_word((uint16_t*)EE_MASTER_SALT, salt);
		if(eeprom_read_word((const uint16_t*)EE_MASTER_SALT) == salt)
		{
			if(salt > 32768)
				clprintW("master salt at %d%%", salt/328);
			return salt;
		}
	}
	//eeprom seems to have gone bad or the salt is run out
	clprintE("master salt overused");
	fatalError = PSTR("master salt overused");
	return 0;
}

void keySetup(client_t *c, uint8_t remote_addr, uint16_t negotiated_salt)
{
//worst key derivation function eva!!!
//replace later
	uint8_t buf[16];
	aes128_ctx_t client_aes_ctx;

	memset(buf+5, 0, 16-5);
	buf[0] = negotiated_salt;
	buf[1] = negotiated_salt>>8;
	buf[2] = min(mrbus_dev_addr, remote_addr);
	buf[3] = max(mrbus_dev_addr, remote_addr);
	buf[4] = 1;
	aes128_enc(buf, &master_aes_ctx);
	aes128_init(buf, &client_aes_ctx);
	cmac_aes_init(&client_aes_ctx, &c->cmac_ctx);

	memset(buf+5, 0, 16-5);
	buf[0] = negotiated_salt;
	buf[1] = negotiated_salt>>8;
	buf[2] = min(mrbus_dev_addr, remote_addr);
	buf[3] = max(mrbus_dev_addr, remote_addr);
	buf[4] = 2;
	aes128_enc(buf, &master_aes_ctx);
	memcpy(c->eax_nonce, buf, 4);
	
	marktime(c);

}


void RDP_USER_DATA_RECV(client_t *c, uint8_t channel, uint16_t len, uint8_t *data)
{
	switch(channel)
	{
	case 1:	
		while(len)
		{
			vfd_putchar(*data, NULL);
			len--;
			data++;
		}
		break;
	}
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
		c->recvBufferCount-=len+2;
	}
}

void RDP_ACKNACK(client_t *c, uint32_t pktNum, uint8_t an)
{
	if (c->minack<=pktNum)
	{
		uint8_t x=pktNum-c->minack;
		if(an)
		{
			c->acks|=1<<x;
			c->nacks&=~(1<<x);
		}
		else
		{
			c->acks&=~(1<<x);
			c->nacks|=1<<x;
		}
	}
	else
	{
		uint8_t x=c->minack-pktNum;
		c->acks<<=x;
		c->nacks<<=x;

		if(an)
			c->acks|=1;
		else
			c->nacks|=1;
	}
}
#define RDP_ACK(c,p) RDP_ACKNACK(c,p,1)
#define RDP_NACK(c,p) RDP_ACKNACK(c,p,0)


uint8_t RDPTXneeded(void)
{
	for (client_t *c=clients;c<clients+MAXCLIENTS; c++)
		if(c->addr && (c->acks||c->nacks))
			return 1;

	return 0;
}

uint8_t RDPTX()
{
static client_t *c=clients;//save it off so that we can round-robbin to make sure no one gets starved
uint8_t i;
	for (i=0;i<MAXCLIENTS; i++, c++)
	{
		if(c>=clients+MAXCLIENTS)
			c=clients;
		if(c->addr==0)
			continue;
	
		if(c->acks||c->nacks)
			{

//here				
				return 0;
			}
	}
	return 1;


/*RDP_tx_pkt_queue_t txPktBuffer[RDP_MAX_TX_OVERLOAD];
	uint32_t pktNum;
	uint16_t timer;
	uint8_t mrbPkt[MRBUS_BUFFER_SIZE];
*/


}

void cheesyPinGet(uint8_t id, uint8_t *rlen, uint32_t *rpin)
{
	if (id >= sizeof(PINS)/sizeof(*PINS))
	{
		*rlen=0;
		*rpin=0;
	}
	else
	{
		*rlen=PINS[id].len;
		*rpin=PINS[id].num;
	}

}
void cheesyPinGetx(uint8_t id, uint8_t *rlen, uint32_t *rpin)
{
	uint8_t len;
	uint32_t pin=eeprom_read_dword((const uint32_t*)(100+4*id));
	for(len=9;len&&!(pin&1);len--,pin>>=1)
		;
	pin>>=1;
	*rlen=len;
	*rpin=pin;
}

void cheesyPinSet(uint8_t id, uint8_t len, uint32_t pin)
{
	pin<<=1;
	pin|=1;
	pin<<=9-len;
	eeprom_write_dword((uint32_t*)(100+4*id), pin);
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
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)rxBuffer[6], rxBuffer[7]);
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = rxBuffer[7];
		if (MRBUS_EE_DEVICE_ADDR == rxBuffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
send:
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;	
	}
	else if ('R' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'r';
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rxBuffer[6]);			
		goto send;
	}
	else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Version
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
		goto send;
	}
	else if ('X' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		reset();
	}
	else if ('Z' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// aes test
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
		goto send;
	}
	else if ('1' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// omac test
		txBuffer[MRBUS_PKT_TYPE] = '2';
		txBuffer[MRBUS_PKT_LEN] = 20;
		uint8_t l = rxBuffer[MRBUS_PKT_LEN]-6;
		uint8_t buf[16];
		cmac_aes(&master_cmac_ctx, buf, rxBuffer+MRBUS_PKT_TYPE+1, l);
		for(i=0;i<14;i++)
		  txBuffer[i+6] = buf[i];
		goto send;
	}
	else if ('3' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// eax test
		txBuffer[MRBUS_PKT_TYPE] = '4';
		uint8_t nl = rxBuffer[6];
		uint8_t hl = (nl>>4)&0xf;
		nl&=0xf;
		uint8_t dl=rxBuffer[MRBUS_PKT_LEN]-7-nl-hl;
		if (dl+3>14)
			dl=14-3;
		eax_aes_enc(&master_cmac_ctx, txBuffer+6, 3, rxBuffer+7, nl, rxBuffer+7+nl, hl, rxBuffer+7+nl+hl, dl);
		txBuffer[MRBUS_PKT_LEN] = 6+dl+3;
		goto send;
	}
	else if ((int8_t)rxBuffer[MRBUS_PKT_TYPE] < RDP_SEQ_N+1-128) 
	{
		//secure channel packet
		client_t *c=clients;
		uint8_t i;
		for (i=0;i<MAXCLIENTS; i++, c++)
			if(c->addr==rxBuffer[MRBUS_PKT_SRC])
				break;
		if(i>=MAXCLIENTS)
		{
			for (i=0, c=clients;i<MAXCLIENTS; i++, c++)
				if(c->addr==0)
					break;
			if(i>=MAXCLIENTS)
			{
closeRDPSend:
				//no open connection slots. close.
				txBuffer[MRBUS_PKT_LEN] = 7;
				txBuffer[MRBUS_PKT_TYPE] = RDP_SEQ_N-128;
				txBuffer[6]  = RDP_PKT_RST;
				goto send;
			}
			//we don't have a conversation with this client, and we have an open slot
			//startup. equiventent to the listen state in RUDP
			if ((int8_t)rxBuffer[MRBUS_PKT_TYPE] == RDP_SEQ_N-128 && rxBuffer[MRBUS_PKT_LEN] == 7 && rxBuffer[6]==RDP_PKT_SYN)
			{
restartRDP:
				//syn recvd
				memset(c, 0, sizeof(client_t));
				//taken care of by memset 0: c->state = RDP_SYN_RCVD;
				c->addr = rxBuffer[MRBUS_PKT_SRC];
				c->maxpktsize=20;
			}
			else
				goto PktIgnore;
		}

		if ((int8_t)rxBuffer[MRBUS_PKT_TYPE] == RDP_SEQ_N-128 && rxBuffer[MRBUS_PKT_LEN] == 7 && rxBuffer[6]==RDP_PKT_RST)
		{//passive close
			c->addr=0;
			goto PktIgnore;
		}
		else if(c->state == RDP_SYN_RCVD)
		{
			if ((int8_t)rxBuffer[MRBUS_PKT_TYPE] == RDP_SEQ_N-128 && rxBuffer[MRBUS_PKT_LEN] == 7 && rxBuffer[6]==RDP_PKT_SYN)
			{
				//send syn,ack
				txBuffer[MRBUS_PKT_LEN] = 7;
				txBuffer[MRBUS_PKT_TYPE] = RDP_SEQ_N-128;
				txBuffer[6]  = RDP_PKT_SYNACK;
				marktime(c);
				goto send;
			}
			else if ((int8_t)rxBuffer[MRBUS_PKT_TYPE] == RDP_SEQ_N-128 && rxBuffer[MRBUS_PKT_LEN] == 7 && rxBuffer[6]==RDP_PKT_ACK)
			{
				//send ack
				txBuffer[MRBUS_PKT_LEN] = 7;
				txBuffer[MRBUS_PKT_TYPE] = RDP_SEQ_N-128;
				txBuffer[6]  = RDP_PKT_ACK;
				marktime(c);
				c->state = RDP_OPEN;
				goto send;
			}
			else
			{//active close
				c->addr=0;
				goto closeRDPSend;
			}
		}
		else if(c->state == RDP_OPEN)
		{
			if ((int8_t)rxBuffer[MRBUS_PKT_TYPE] == RDP_SEQ_N-128 && rxBuffer[MRBUS_PKT_LEN] == 7 && rxBuffer[6]==RDP_PKT_SYN)
			{
				//got a mis-placed syn. assume we missed a RST
				goto restartRDP;
			}
			else if ((int8_t)rxBuffer[MRBUS_PKT_TYPE] == RDP_SEQ_N-128 && rxBuffer[MRBUS_PKT_LEN] == 7 && rxBuffer[6]==RDP_PKT_ACK)
			{
				marktime(c);
				goto PktIgnore;
			}
			else if ((int8_t)rxBuffer[MRBUS_PKT_TYPE] == RDP_SEQ_N-128 && rxBuffer[MRBUS_PKT_LEN] >= 11 && rxBuffer[6]==RDP_PKT_DATA)
			{
				//data in control packet
				//buf at rxBuffer+6+1+4
				//buf len is rxBuffer[MRBUS_PKT_LEN]-6-1-4
				//seq num *(uint32_t*)(rxBuffer+7)

				marktime(c);

				uint32_t pktNum = *(uint32_t*)(rxBuffer+7);
				uint8_t sz=rxBuffer[MRBUS_PKT_LEN]-6-1-4;
				//ok, data recvd pktNum, in rxBuffer+6, size sz
				if(c->recvPktCounter<pktNum)
					c->recvPktCounter=pktNum;

//{int i;for(i=7;i<rxBuffer[MRBUS_PKT_LEN];i++)printf("%02x ", rxBuffer[i]);}
				if(pktNum-c->recvLastPkt>RDP_RECV_OVERLOAD)
				{
					RDP_NACK(c, pktNum);
					goto PktIgnore;
				}

				if(pktNum<=c->recvLastPkt || c->recvMarkers&(1<<(pktNum-c->recvLastPkt)))//we don't need this again
				{
					RDP_ACK(c, pktNum);
					goto PktIgnore;
				}
			
				RDP_buffer_slot_t *p = (void*)(c->recvBuffer+c->recvBufferCount); 
				int8_t x = pktNum-c->recvLastPkt-1;
//cvprintf("[%lu %u]", pktNum,c->recvPktFirstMissing);

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
						++c->recvLastPkt;
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

					c->recvMarkers|=(1<<(pktNum-c->recvLastPkt));
					(p-1)->pktsInData++;
					(p-1)->dataLen+=sz;
				}
				RDP_ACK(c, pktNum);
				memcpy(p->missingPkt, rxBuffer+6+1+4, sz);
				memmove((uint8_t*)p+sz, p->collapsedData, c->recvBuffer+RDP_RECV_PKT_BUFFER_SIZE-p->collapsedData);
				memset(c->recvBuffer+RDP_RECV_PKT_BUFFER_SIZE-(p->collapsedData-((uint8_t*)p+sz)), 0, p->collapsedData-((uint8_t*)p+sz));

				RDP_DATA_RECV(c);
			}

		}
	
	
	}
	//*************** END PACKET HANDLER  ***************
}


void init(void)
{
	//set all pins to input with pull ups..
	MCUCR=0;
	PORTB = PORTC = PORTD = 0xff;
	DDRB = DDRC = DDRD = 0;


	// Set MOSI, SCK, SSs, and reset to output.  also set PB2, which is SPI SS. needs to be output for master mode to work.  mabe move one of our SSs here later 
	DDRB = (1<<NFC_SS)|(1<<VFD_RESET__NFC_WAKE)|(1<<MOSI)|(1<<SCK)|(1<<SPKR_OC1B);
	DDRD = (1<<VFD_SS);
	PORTD = ~(1<<VFD_SS);

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
	

	//turn on the pin change interupt from the VFD busy flag
	PCMSK2 = 1<<VFD_BUSY;
	PCICR = 1<<PCIE2;


	KBD_ringBufferInitialize(&KBD_ringBuffer);
	VFD_ringBufferInitialize(&VFD_ringBuffer);
	SPIState = (SPIStates_t){SPI_VFD, VFD_BUSYWAIT, NFC_POLL, 10};

	deciSecs=0;
	ticks50khz=0;

	//set up Timer/Counter1 Compare Match B to toggle for speaker
	TCCR1A = (1<<COM1B0);
	TCCR1B = (1<<WGM12)|(1<<CS11);// for 20Mhz/8
	TCCR1C = 0;
  OCR1B=0;

	stdout = stdin = &vfd_str;
	stderr = &log_str;

	_delay_ms(2);
	PORTD &= ~(1<<VFD_RESET__NFC_WAKE);
	_delay_ms(2);
	PORTD |= (1<<VFD_RESET__NFC_WAKE);

	initialize50kHzTimer();

	cvprintf(VFDCOMMAND_INITIALIZE "Startup: Init");

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbusInit();


	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}

	aes128_init(MASTERKEY, &master_aes_ctx);
	cmac_aes_init(&master_aes_ctx, &master_cmac_ctx);

	for (uint8_t i=0;i<MAXCLIENTS; i++)
		memset(clients+i, 0, sizeof(client_t));

	sei();	
}


void updateCheesyUI(int8_t state, uint8_t id, uint8_t key, uint8_t time, uint8_t newstate)
{
	int length = 0;
	char buf[100];

	if(newstate)
		length += sprintf_P (buf+length, PSTR(VFDCOMMAND_CLEARHOME VFDCOMMAND_BRIGHTNESS(8) ));
	else
		length += sprintf_P (buf+length, PSTR(VFDCOMMAND_HOME VFDCOMMAND_BRIGHTNESS(8) ));

	if (state >=0)
	{
		uint8_t r = 128;
		uint8_t g = 128;
		uint8_t b = 255;

		length += sprintf_P (buf+length, PSTR("\x1fL\x10%c%c%cEnter ID & Pass: "), b & 0xf0, g & 0xf0, r & 0xf0);
		if (state>=1)
			length += sprintf_P (buf+length, PSTR("%d "), state>=2?id/10:id);
		else
			length += sprintf_P (buf+length, PSTR("_ "));

		if (state>=2)
			length += sprintf_P (buf+length, PSTR("%d "), id%10);
		else
			length += sprintf_P (buf+length, PSTR("_ "));

		int8_t i;
		for(i=2;i+1<state;i++)
			length += sprintf_P (buf+length, PSTR("* "));
		if (state>=3)
		{
			if (key==0)
				key='*';
			length += sprintf_P (buf+length, PSTR("%c"), key);
		}
	}
	else if (state==-1)
		length += sprintf_P (buf+length, PSTR("\x1fL\x10%c%c%cIncorrect"), 16 & 0xf0, 16 & 0xf0, 255 & 0xf0);
	else if (state==-2)
	{
		length += sprintf_P (buf+length, PSTR("\x1fL\x10%c%c%c Open  ["), 16 & 0xf0, 255 & 0xf0, 16 & 0xf0);

		uint8_t i;
		for(i=0;i<10;i++)
			buf[length++]=i<time*10/LOCK_OPEN_TIME?'*':' ';
		length += sprintf_P (buf+length, PSTR("]\n   Push then Pull"));
	}
	else
		length += sprintf_P (buf+length, PSTR(VFDCOMMAND_CLEARHOME "\x1fL\x10%c%c%cUI Err"), 255 & 0xf0, 255 & 0xf0, 255 & 0xf0);

	cvprintf("%s", buf);
}

uint8_t cheesyPinTest(uint8_t id, uint8_t len, uint32_t pin)
{
	if (len<4)
		return 1;
	uint8_t clen;
	uint32_t cpin;
	cheesyPinGet(id, &clen, &cpin);
	if ((clen!=len)||(cpin!=pin))
		return 2;

	return 0;

}


void cheesyUI()
{
	static int8_t state=0;
	static uint8_t id=0;
	static uint32_t pin=0;
	static int16_t timer=0;
	static int16_t lastupdate=0;
	static uint16_t fail_timeout=30;//warning... the doubling can be subverted by forcing a reboot

	if (KBD_ringBufferDepth(&KBD_ringBuffer))
	{
		unsigned char key = KBD_ringBufferPopNonBlocking(&KBD_ringBuffer);
		if ((state==-1) && (deciSecs - timer < fail_timeout))
			return;
		if (state<0)//in open or fail message state and button pushed, get back to 0
		{	
clear:
			state=0;pin=0;id=0;
			updateCheesyUI(state, id, 0, 0, 1);
			return;
		}
		timer=deciSecs;
		if(key=='*')
			goto clear;
		else if(key=='#')
		{
			if (state <= 2)
				goto clear;
			if (cheesyPinTest(id, state-2, pin))
			{
				state=-1;pin=0;id=0;
				updateCheesyUI(state, id, 0, deciSecs - timer, 1);
				set_tone_pattern(TONE_FAIL);
				fail_timeout*=2;
				if (fail_timeout>200)
					fail_timeout=200;
				return;
			}

		  //succeed
			state=-2;pin=0;id=0;
			updateCheesyUI(state, id, 0, deciSecs - timer, 1);
			set_tone_pattern(TONE_OPEN);
			fail_timeout=30;
			uint8_t txBuffer[8];
			txBuffer[MRBUS_PKT_LEN] = 8;
			txBuffer[MRBUS_PKT_TYPE] = 'C';
			txBuffer[6]  = LOCK_OPEN_TIME;
			txBuffer[7]  = 1;
			txBuffer[MRBUS_PKT_DEST] = 0x10;
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			return;
		}
		else if(state<2) 
		{
			id=id*10+(key-'0');
			state++;
			updateCheesyUI(state, id, 0, 0, 0);
			return;
		}
		else if (state <11)
		{
			pin=pin*10+(key-'0');
			state++;
			updateCheesyUI(state, id, key, 0, 0);
			return;
		}
		  state=-1;pin=0;id=0;
			updateCheesyUI(state, id, 0, deciSecs - timer, 1);
			set_tone_pattern(TONE_TIMEOUT);
			return;
	}

	if ( ((state >0) && (deciSecs - timer > 50))
		|| ((state ==-1) && (deciSecs - timer > (fail_timeout<50?50:fail_timeout))) 
		|| ((state ==-2) && (deciSecs - timer > LOCK_OPEN_TIME))
	)
	{
		state=0;pin=0;id=0;
		set_tone_pattern(TONE_TIMEOUT);
		updateCheesyUI(state, id, 0, deciSecs - timer, 1);
		return;
	}

	if(deciSecs - lastupdate > 5)
	{
		lastupdate = deciSecs;
		updateCheesyUI(state, id, 0, deciSecs - timer, 0);
		fail_timeout--;
		if (fail_timeout<10)
			fail_timeout=10;
	}

}




int main(void)
{
ct_assert( (RDP_RECV_PKT_BUFFER_SIZE)<256 );//just a test.  if this fails, you have chosen buffer sizes that require you to change client_t.recvBufferCount to a uint16_t. I can't do it automatically since sizeof isn't available to the preprocessor

		

	// Application initialization
	init();

	cvprintf(VFDCOMMAND_CLEARHOME "Starting Up...");


	while (1)
	{
		wdt_reset();
		cheesyUI();

		if(fatalError)
		{
			cvprintf(VFDCOMMAND_CLEARHOME);
			printf_P(fatalError);		
		}

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
			PktHandler();

		// Handle any packets outgoing packets in the simple queue or in the RDP sender. if both want to send, use the RDPvsQueue flag to round-robbin them
		if (RDPTXneeded() && (RDPvsQueue || 0==mrbusPktQueueDepth(&mrbusTxQueue)))
		{
			if (!RDPTX())
				RDPvsQueue=0;
		}
		else if (mrbusPktQueueDepth(&mrbusTxQueue))
		{
			if (!mrbusTransmit())
				RDPvsQueue=1;
		}

	}
}



