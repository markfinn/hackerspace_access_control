/*************************************************************************
Title:    MRBus AVR Template
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan Holmes

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
#include <stdbool>
#include <stdarg.h>

#ifdef MRBEE
// If wireless, redefine the common variables and functions
#include "mrbee.h"
#define mrbus_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define MRBUS_TX_PKT_READY MRBEE_TX_PKT_READY
#define MRBUS_RX_PKT_READY MRBEE_RX_PKT_READY
#define mrbux_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define mrbusInit mrbeeInit
#define mrbusPacketTransmit mrbeePacketTransmit
#endif

#include "mrbus.h"


#define KEYPADADDR (37<<1)
#define RPOOLSIZE 30

uint8_t randpool[RPOOLSIZE], randpoolstart=0, randpoolend=0;

uint16_t lastkeytime=0;
uint32_t userid, userpin;


uint8_t mrbus_dev_addr = 0;
uint8_t mrbus_master_addr = 0;


// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)

uint16_t centisecs=0, i2cEntropyTimer=0;

void initialize100HzTimer(void)
{
	TCNT0 = 0;
	OCR0A = 0xC2;
	centisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	centisecs++;
}

void restartEncryption(){
//destroy the encryption tunnel if up or broken, then flag as needing a re-build in the state machine
//not done
}
void addEntropy(uint8_t val)
{
// uses jenkins_one_at_a_time_hash
	static uint32_t hash=0;
	static uint8_t c=0;
	static uint8_t h=0;

  hash += val;
  hash += (hash << 10);
  hash ^= (hash >> 6);

	if(val!=0)
		c=(c+1)%13;

	if(0==c)
	{
		hash += (hash << 3);
		hash ^= (hash >> 11);
		hash += (hash << 15);

		uint8_t i;
		if(randpoolstart == randpoolend)
			for(i=0;i<RPOOLSIZE/2+(RPOOLSIZE%2?0:1);i++, h++)
				randpool[h] ^= hash>>(7*(i%4));
		else
			for(i=0;i<3&&randpoolstart != randpoolend; i++, randpoolend=(randpoolend+1)%RPOOLSIZE)
				randpool[randpoolend]=hash>>(8*i);
	}
}

//todo save a bit of entropy to the eeprom every few hours.  reload it on start up.


enum TWISTATE { TWI_IDLE_SUCCESS=0, TWI_IDLE_FAILURE, TWI_BUSY };
enum TWISTATE twistate = TWI_IDLE_SUCCESS;
uint8_t twi_i, twi_wcount, twi_rcount, *twi_waddr, *twi_raddr;

ISR(TWI_vect){
	if(badone) blah;
	switch(TWSR & 0xF8)
	{
	case 0x08://start
	case 0x10://repeated start
		if (wcount)
			TWDR = KEYPADADDR&^1;
		else
			TWDR = KEYPADADDR|1;
		TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		return;
	case 0x18://SLA+W has been transmitted; ACK has been received
		TWDR = twi_waddr[0];
		twi_i=1;
		TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		return;
	case 0x20://SLA+W has been transmitted; NOT ACK has been received
		TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		twistate = TWI_IDLE_FAILURE;
		return;
	case 0x28://Data byte has been transmitted; ACK has been received
		if(twi_i>=twi_wcount)
		{//done writing
			if(0==twi_rcount)
			{
				//stop
				TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(1<<TWEN)|(1<<TWIE);
				twistate = TWI_IDLE_SUCCESS;
			}
			else
			{
				//repeated start, now as a read
				twi_wcount=0;
				TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
			}
			return;
		}
		TWDR = twi_waddr[twi_i++];
		TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		return;
	case 0x30://Data byte has been transmitted; NOT ACK has been received
		TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		twistate = TWI_IDLE_FAILURE;
		return;
	case 0x38://Arbitration lost in [write: SLA+W or data bytes] or [read: SLA+R or NOT ACK bit]
				TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		return;
	case 0x40://SLA+R has been transmitted; ACK has been received
		twi_i=0;
		if(twi_rcount <= 1)  //read with nack
			TWCR = (1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		else	//read with ack
			TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		return;
	case 0x48://SLA+R has been transmitted; NOT ACK has been received
		TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		twistate = TWI_IDLE_FAILURE;
		return;
	case 0x50://data byte received, ACK transmitted
		twi_raddr[twi_i++]=TWDR;
		if(twi_i+1 < twi_rcount)  //read with nack
			TWCR = (1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		else	//read with ack
			TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		return;
	case 0x58://data byte received, NACK transmitted
		TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		twistate = TWI_IDLE_SUCCESS;
		return;
	case 0xF8://not available
		return;
	default:
		TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		twistate = TWI_IDLE_FAILURE;
		return;
	}
}


bool i2c_setup(uint8_t wcount, uint8_t *wp, uint8_t rcount, uint8_t *rp)
{
	if(wcount == 0 && rcount == 0 || twistate == TWI_BUSY || TWCR&(1<<TWSTO))
		return true;

	twi_wcount=wcount;
	twi_rcount=rcount;
	twi_waddr=wp;
	twi_raddr=rp;

	twistate = TWI_BUSY;

	//send start
	TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
}


bool readI2CKeypadStart()
{
	if(i2c_setup(1, (uint8_t[]){1}, 2, &i2c_raw))
	{
		addEntropy(-1);
		return true;
	}
	return false;
}

int16_t asdfreadI2CKeypadStart()
{
	if(i2c_setup(1, (uint8_t[]){1}, 2, key_raw))
	{
		addEntropy(-1);
		return -1;
	}
	addEntropy(k1);
	addEntropy(k2);
	return ((k1&0x0f)<<8)|k2;
}


bool readi2centropyStart()
{
	return i2c_setup(1, (uint8_t[]){2}, 1, key_raw);
}

uint8_t asdfreadi2centropyreg()
{
	uint8_t v;
	if(i2c_writeread(KEYPADADDR, 1, 1, 2, &v))
		return 0;
	return v;
}



bool seti2cledStart(bool status, bool keypad)
{
	if(twistate == TWI_BUSY)
		return true;
	key_raw[0]=2;
	key_raw[1]=status?2:0 | keypad?1:0;
	i2c_setup(2, key_raw, 0, NULL);
	return false;
}



bool beepi2CStart(uint8_t halfperiod, uint16_t periods)
{
	if(twistate == TWI_BUSY)
		return true;
	key_raw[0]=1;
	key_raw[1]=halfperiod;
	key_raw[2]=periods>>8;
	key_raw[3]=periods;
	i2c_setup(4, key_raw, 0, NULL);
	return false;
}


void beepErr(){
	leds(1,0);
	beep(64,1000);
	leds(0,1);
	beep(64,1000);
	leds(1,0);
	beep(64,1000);
	leds(0,1);
	beep(64,1000);
	leds(1,0);
	beep(64,1000);
	leds(0,1);
	beep(64,1000);
}

void beepKey(){	
	leds(0,0);
	beep(32,300);
	leds(0,1);
}

void beepUnlock(){
	leds(1,0);
	beep(45,300);
	leds(1,1);
	beep(28,300);
	leds(0,1);
}


void beepAck(){
	leds(1,0);
	beep(32,600);
	leds(0,1);

}


void lock(){}
void unlock(){}


void encryptBuffer()
{
	uint8_t i;
	uint8_t l = mrbus_tx_buffer[MRBUS_PKT_LEN];
	for(i = l; i>MRBUS_PKT_TYPE; i--)
		mrbus_tx_buffer[i] = mrbus_tx_buffer[i-1]^0x55;
	mrbus_tx_buffer[MRBUS_PKT_TYPE] = '$';
	mrbus_tx_buffer[MRBUS_PKT_LEN]=l+1;
}

void decryptBuffer()
{
	uint8_t i;
	uint8_t l = mrbus_tx_buffer[MRBUS_PKT_LEN]-1;
	for(i = MRBUS_PKT_TYPE; i<l; i++)
		mrbus_tx_buffer[i] = mrbus_tx_buffer[i+1]^0x55;
	mrbus_tx_buffer[MRBUS_PKT_TYPE] = '$';
	mrbus_tx_buffer[MRBUS_PKT_LEN]=l;
}


void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;

	addEntropy(centisecs>>8);
	addEntropy(centisecs);
	addEntropy(TCNT0);

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (mrbus_rx_buffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != mrbus_rx_buffer[MRBUS_PKT_DEST] && mrbus_dev_addr != mrbus_rx_buffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<mrbus_rx_buffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, mrbus_rx_buffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
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
	
	if ('A' == mrbus_rx_buffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 6;
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'a';
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	} 
	else if ('W' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6], mrbus_rx_buffer[7]);
		mrbus_tx_buffer[6] = mrbus_rx_buffer[6];
		mrbus_tx_buffer[7] = mrbus_rx_buffer[7];
		if (MRBUS_EE_DEVICE_ADDR == mrbus_rx_buffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	
	}
	else if ('R' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'r';
		mrbus_tx_buffer[6] = mrbus_rx_buffer[6];
		mrbus_tx_buffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6]);			
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	}
	else if ('V' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// Version
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 17;
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'v';
		mrbus_tx_buffer[6]  = MRBUS_VERSION_WIRED;
		mrbus_tx_buffer[7]  = 0; // Software Revision
		mrbus_tx_buffer[8]  = 0; // Software Revision
		mrbus_tx_buffer[9]  = 0; // Software Revision
		mrbus_tx_buffer[10]  = 0; // Hardware Major Revision
		mrbus_tx_buffer[11]  = 0; // Hardware Minor Revision
		mrbus_tx_buffer[12] = 'L';
		mrbus_tx_buffer[13] = 'O';
		mrbus_tx_buffer[14] = 'C';
		mrbus_tx_buffer[15] = 'K';
		mrbus_tx_buffer[16] = 'A';
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	}
	else if ('X' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
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
	else if ('$' == mrbus_rx_buffer[MRBUS_PKT_TYPE] && mrbus_dev_addr == mrbus_rx_buffer[MRBUS_PKT_DEST] && mrbus_master_addr == mrbus_rx_buffer[MRBUS_PKT_SRC]) 
	{
		decryptBuffer();
		if ('a' == mrbus_rx_buffer[MRBUS_PKT_TYPE])
		{
			if (keystate == KEYSTATE_AUTHWAIT)
				keystate = (mrbus_rx_buffer[6]==1)?KEYSTATE_SUCCESS:KEYSTATE_FAILURE;
			else
				restartEncryption();
		}
		goto PktIgnore;
	}
	else if ('Q' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		switch(mrbus_rx_buffer[6])
		{
			case 1:
				leds(mrbus_rx_buffer[7],mrbus_rx_buffer[8]);
				break;
			case 2:
				beep(mrbus_rx_buffer[7],(mrbus_rx_buffer[8]<<8)|mrbus_rx_buffer[9]);
				break;
			case 3:
				beepErr();
				break;
			case 4:
				beepKey();
				break;
			case 5:
				beepAck();
				break;
			case 6:
				beepUnlock();
				break;
			case 7:
				lock();
				break;
			case 8:
				unlock();
				break;
		}
		goto PktIgnore;
	}

	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	mrbus_state &= (~MRBUS_RX_PKT_READY);
	return;	
}

void init(void)
{
#define SCL_CLOCK 40000
#define TWBR_VAL (((F_CPU/SCL_CLOCK)-16)/2)
#if TWBR_VAL<=10 || TWBR_VAL > 255
#error change SCL prescaler
#endif

	PORTA=0xff;
	PORTB=0xff;
	PORTC=0xff;
	PORTD=0xff;


	//initialize TWI I2C 40 kHz clock, TWPS = 0 => prescaler = 1 */
	TWSR = 0;//prescaler = 1
	TWBR = TWBR_VAL;
	//Don't respond to any addresses as a slave
	TWAR = TWBR_VAL;
	//enable the TWI
	TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);



	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
	wdt_reset();
	wdt_disable();

	// Initialize MRBus address from EEPROM address 1
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)0);
	mrbus_master_addr = eeprom_read_byte((uint8_t*)1);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
	}
	if (0xFF == mrbus_master_addr || 0x00 == mrbus_master_addr)
	{
		mrbus_master_addr = mrbus_dev_addr;
	}

//		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);

	restartEncryption();
	leds(0,0);

}

/*
	returns the current keypad bitmap.  the high bit set means failed to read.
*/


int8_t readKeypad()
{
	static uint16_t last=0;
	static uint16_t debouce=0;
	uint16_t value = readI2CKeypad();
	uint8_t ret=15;


	//if keyboard read error, return no change.
	if(value<0)
		return INT8_MIN;

	//debounce the keypad by only reading so often.
	//note that we still call readI2CKeypad() so that the entropy generation is maximized.
	//also note that this means two or more keys pressed at the same will come in as separate presses at 50ms intervals
	if(centisecs-debouce < 5)
		return INT8_MIN;
	debouce = centisecs;

	//get the change
	value ^= last;

	if(value==0)
		return INT8_MIN;

	//mask all but the lowest bit in the change	
	value &= -(int16_t)value;

	//binary search for the set bit position
	if (value & 0x00ff) ret -= 8; //0000 0000 1111 1111
	if (value & 0x0f0f) ret -= 4; //0000 1111 0000 1111
	if (value & 0x3333) ret -= 2; //0011 0011 0011 0011
	if (value & 0x5555) ret -= 1; //0101 0101 0101 0101

	//if this is a key release, negate.
	if(value&last)
		ret = -ret;

	//mark this key change as known
	last ^= value;

	return ret;
}




void sendEncryptedAuthRequest(uint32_t userid, uint32_t userpin)
{
	mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
	mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_master_addr;
	mrbus_tx_buffer[MRBUS_PKT_LEN] = 14;
	mrbus_tx_buffer[5] = 'A';
	mrbus_tx_buffer[6] = userid>>24;
	mrbus_tx_buffer[7] = userid>>16;
	mrbus_tx_buffer[8] = userid>>8;
	mrbus_tx_buffer[9] = userid;
	mrbus_tx_buffer[10] = userpin>>24;
	mrbus_tx_buffer[11] = userpin>>16;
	mrbus_tx_buffer[12] = userpin>>8;
	mrbus_tx_buffer[13] = userpin;
	encryptBuffer();
	mrbus_state |= MRBUS_TX_PKT_READY;
}


void service_mrbus()
{

	// Handle any packets that may have come in
	if (mrbus_state & MRBUS_RX_PKT_READY)
		PktHandler();
		
	// If we have a packet to be transmitted, try to send it here
	if(mrbus_state & MRBUS_TX_PKT_READY)
	{
		if (0 == mrbusPacketTransmit())
			mrbus_state &= ~(MRBUS_TX_PKT_READY);
	}

}

void service_ui()
{
enum KEYSTATE { KEYSTATE_WAITING=0, KEYSTATE_USERID, KEYSTATE_USERPIN, KEYSTATE_AUTHWAIT, KEYSTATE_SUCCESS, KEYSTATE_UNLOCKED, KEYSTATE_FAILURE };
enum KEYSTATE keystate = KEYSTATE_WAITING;

	if (keystate !=KEYSTATE_WAITING && keystate < KEYSTATE_AUTHWAIT && centisecs-lastkeytime > 400)
	{
		beepErr();
		keystate=0;
	}


	if (centisecs - i2cEntropyTimer > 5)
		addEntropy(readi2centropyreg());

	int8_t key;
	switch (keystate)
	{
		default:
		case KEYSTATE_WAITING:
			leds(0,1);
			lock();
			key = readKeypad();
			if (key!=INT8_MIN)
			{
				lastkeytime = centisecs;
				if(key < 10)
				{
					beepKey();
					userid = key;
				}
				else
				{
					beepErr();
					keystate=KEYSTATE_WAITING;
				}
			}

		case KEYSTATE_USERID://reading userid
			key = readKeypad();
			if (key!=INT8_MIN)
			{
				lastkeytime = centisecs;
				if(key == 10)
				{
					beepAck();
					keystate++;
					userpin=0;
				}
				else if(userid < 100000000)
				{
					beepKey();
					userid = userid*10 + key;
				}
				else
				{
					beepErr();
					keystate=KEYSTATE_WAITING;
				}
			}
		break;

		case KEYSTATE_USERPIN://reading pin
			key = readKeypad();
			if (key!=INT8_MIN)
			{
				lastkeytime = centisecs;
				if(key == 10)
				{
					beepAck();
					sendEncryptedAuthRequest(userid, userpin);
					keystate++;
				}
				else
				{
					beepKey();
					userpin = userpin*10 + key;
				}
			}
		break;

		case KEYSTATE_AUTHWAIT://waiting for response
			//state will be changed by auth handler
			if(centisecs-lastkeytime < 200)
				break;
			restartEncryption();
			lock();
			beepErr();
			keystate=KEYSTATE_WAITING;
		break;

		case KEYSTATE_SUCCESS:
			lastkeytime = centisecs;
			unlock();
			beepUnlock();
			keystate++;
		break;

		case KEYSTATE_UNLOCKED:
			if(centisecs-lastkeytime < 500)
				break;
			lock();
			beepAck();
			keystate=KEYSTATE_WAITING;
		break;

		case KEYSTATE_FAILURE:
			lock();
			beepErr();
			keystate=KEYSTATE_WAITING;
		break;

	}

}
int main(void)
{
	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusInit();

	sei();	

	while (1)
	{
		addEntropy(centisecs>>8);
		addEntropy(centisecs);
		addEntropy(TCNT0);

		//not done add avr watchdog timer to entropy

		service_mrbus();

		service_ui();

		service_i2c_beep();
		service_i2c_led();
		service_i2c_keypad();
		service_i2c_entropy();
	}
}




