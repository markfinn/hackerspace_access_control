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
#include <stdbool.h>
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

uint16_t lastkeytime=0, spiTimer;
uint32_t userid, userpin;
#define AUTH_NONE 0
#define AUTH_WAIT 1
#define AUTH_READY 2
#define AUTH_TIMEOUT 3
#define AUTH_MASK_STATE (AUTH_NONE|AUTH_WAIT|AUTH_READY|AUTH_TIMEOUT)
#define AUTH_PERMISSION_AUTHENTICATED 4
#define AUTH_PERMISSION_ENTRY 8
#define AUTH_PERMISSION_ADMIN 16
#define AUTH_MASK_PERMISSION (AUTH_PERMISSION_AUTHENTICATED|AUTH_PERMISSION_ENTRY|AUTH_PERMISSION_ADMIN)
uint8_t authstate = AUTH_NONE;


uint8_t mrbus_dev_addr = 0;
uint8_t mrbus_master_addr = 0;


uint16_t currentkeystate=0;


uint16_t centisecs=0, i2cEntropyTimer=0;
enum TWISTATE { TWI_IDLE_SUCCESS=0, TWI_IDLE_FAILURE, TWI_BUSY };
enum TWISTATE twistate = TWI_IDLE_SUCCESS;
uint8_t twi_i, twi_wcount, twi_rcount, *twi_waddr, *twi_raddr, i2c_raw[4];



#define beep(h, p) setI2CBeep(h, p)

#define waitKey() do{\
if(centisecs-lastkeytime < 300)\
	goto WAITING;\
do{key = getKeyPress();}\
	while (key < 0);\
}while(0)











void addEntropy(uint8_t val);
void spin();
bool i2c_setup(uint8_t wcount, uint8_t *wp, uint8_t rcount, uint8_t *rp);
bool waitspi();
bool i2c_run(uint8_t wcount, uint8_t *wp, uint8_t rcount, uint8_t *rp);
int16_t readI2CKeypad();
bool readI2CEntropy();
bool setI2CLed(bool status, bool keypad);
bool setI2CBeep(uint8_t halfperiod, uint16_t periods);
void beepErr(int n);
void beepKey();
void beepUnlock();
void beepAck();
void lock();
void unlock();
void encryptBuffer();
void decryptBuffer();
void PktHandler(void);
void init(void);
int8_t readKeypad();
void sendEncryptedAuthRequest(uint32_t userid, uint32_t userpin);
void service_mrbus();
int8_t getKeyPress();
int main(void);









// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)
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
	authstate = AUTH_NONE;
}
void addEntropy(uint8_t val)
{return;//fix me, this function kills everything
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
		{
			for(i=0;i<RPOOLSIZE/2+(RPOOLSIZE%2?0:1);i++, h++)
				randpool[h] ^= hash>>(7*(i%4));
			randpoolstart=(randpoolstart+1)%RPOOLSIZE;
		}
		else
			for(i=0;i<3&&randpoolstart != randpoolend; i++, randpoolend=(randpoolend+1)%RPOOLSIZE)
				randpool[randpoolend]^=hash>>(8*i);
	}
}

//todo save a bit of entropy to the eeprom every few hours.  reload it on start up.



ISR(TWI_vect){
	switch(TWSR & 0xF8)
	{
	case 0x08://start
	case 0x10://repeated start
		if (twi_wcount)
			TWDR = KEYPADADDR&~1;
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


void spin()
{
	service_mrbus();
	if (centisecs - i2cEntropyTimer > 50 && twistate != TWI_BUSY)
	{
		i2cEntropyTimer = centisecs;
		addEntropy(readI2CEntropy());
	}


}




bool i2c_setup(uint8_t wcount, uint8_t *wp, uint8_t rcount, uint8_t *rp)
{
	if((wcount == 0 && rcount == 0) || twistate == TWI_BUSY || TWCR&(1<<TWSTO))
		return true;

	twi_wcount=wcount;
	twi_rcount=rcount;
	twi_waddr=wp;
	twi_raddr=rp;

	twistate = TWI_BUSY;

	//send start
	TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
	return false;
}




bool waitspi()
{
	spiTimer = centisecs;
	while (twistate == TWI_BUSY)
	{
		spin();
		if (centisecs - spiTimer > 10)
			return true;
	}
	return false;
}

bool i2c_run(uint8_t wcount, uint8_t *wp, uint8_t rcount, uint8_t *rp)
{
	return waitspi() || i2c_setup(wcount, wp, rcount, rp) || waitspi() || twistate != TWI_IDLE_SUCCESS;
}

int16_t readI2CKeypad()
{
	if(i2c_run(1, (uint8_t[]){1}, 2, i2c_raw))
	{
		addEntropy(-1);
		return -1;
	}
	addEntropy(i2c_raw[0]);
	addEntropy(i2c_raw[1]);
	addEntropy(centisecs>>8);
	addEntropy(centisecs);
	addEntropy(TCNT0);
	return ((i2c_raw[0]&0x0f)<<8)|i2c_raw[1];
}


bool readI2CEntropy()
{
	if(i2c_run(1, (uint8_t[]){1}, 2, i2c_raw))
		return 0;
	return i2c_raw[0];
}


bool setI2CLed(bool status, bool keypad)
{
	if (waitspi())
		return true;
	i2c_raw[0]=2;
	i2c_raw[1]=status?2:0 | keypad?1:0;
	i2c_run(2, i2c_raw, 0, NULL);
	return false;
}

#define leds(s, k) setI2CLed(s, k)

bool setI2CBeep(uint8_t halfperiod, uint16_t periods)
{
	if (waitspi())
		return true;
	i2c_raw[0]=1;
	i2c_raw[1]=halfperiod;
	i2c_raw[2]=periods>>8;
	i2c_raw[3]=periods;
	i2c_run(4, i2c_raw, 0, NULL);
	return false;
}



void beepErr(int n){
int i;
	for(i=0;i<n;i++)
	{
		leds(1,0);
		beep(45,1000);
		leds(0,1);
		beep(64,1000);
	}
}

void beepKey(){	
	leds(0,0);
	beep(17,300);
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
		mrbus_tx_buffer[i] = mrbus_tx_buffer[i-1];
//		mrbus_tx_buffer[i] = mrbus_tx_buffer[i-1]^0x55;
	mrbus_tx_buffer[MRBUS_PKT_TYPE] = '$';
	mrbus_tx_buffer[MRBUS_PKT_LEN]=l+1;
}

void decryptBuffer()
{
	uint8_t i;
	uint8_t l = mrbus_tx_buffer[MRBUS_PKT_LEN]-1;
	for(i = MRBUS_PKT_TYPE; i<l; i++)
//		mrbus_tx_buffer[i] = mrbus_tx_buffer[i+1]^0x55;
		mrbus_tx_buffer[i] = mrbus_tx_buffer[i+1];
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
			if (authstate == AUTH_WAIT)
				authstate = mrbus_rx_buffer[6]&AUTH_MASK_PERMISSION|AUTH_READY;
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
				beepErr(mrbus_rx_buffer[7]);
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

	//not sure here
	GPIOR0=0xff;
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


}

/*
	returns the current keypad bitmap.  the high bit set means failed to read.
*/


int8_t readKeypad()
{
	static uint16_t debouce=0;
	static uint16_t valuestore=0;
	uint8_t ret=16;

	uint16_t value = readI2CKeypad();

	//debounce the keypad by only reading so often.
	//note that we still call readI2CKeypad() so that the entropy generation and spinning is maximized.
	//also note that this means two or more keys pressed at the same will come in as separate presses at 50ms intervals

	if (valuestore^currentkeystate)//there's still more change to be converted
		value = valuestore;
	else if(centisecs-debouce < 5 || value < 0)//if keyboard read error, return no change.
		return 0;
	else
		debouce = centisecs;

	//get the change
	value ^= currentkeystate;

	if(value==0)
		return 0;

	//mask all but the lowest bit in the change	
	value &= -(int16_t)value;

	//binary search for the set bit position
	if (value & 0x00ff) ret -= 8; //0000 0000 1111 1111
	if (value & 0x0f0f) ret -= 4; //0000 1111 0000 1111
	if (value & 0x3333) ret -= 2; //0011 0011 0011 0011
	if (value & 0x5555) ret -= 1; //0101 0101 0101 0101

	//if this is a key release, negate.
	if(value&currentkeystate)
		ret = -ret;

	//mark this key change as known
	currentkeystate ^= value;

	return ret;
}




void sendEncryptedAuthRequest(uint32_t userid, uint32_t userpin)
{
	if (authstate&AUTH_MASK_STATE == AUTH_WAIT)
		restartEncryption();
	authstate = AUTH_WAIT;
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


int8_t getKeyPress()
{
	int8_t key = readKeypad();
	if (key <= 0) //key error, non, or release are all ignored
		return -1;

	lastkeytime = centisecs;
	if(key == 10)
		return 0;
	return key;	
}




int main(void)
{
	int8_t key;
	int8_t progmodeenterstate;

	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusInit();

	sei();	

	//not done add avr watchdog timer to entropy


	restartEncryption();
	leds(0,0);
//while(1) spin();


WAITING:

	do{
		leds(0,1);
		lock();
		key = getKeyPress();
	}
	while(key < 0);

	if(key >= 10)
	{
		beepErr(1);
		goto WAITING;
	}

	beepKey();
	userid = key;

	//reading userid
	while(1)
	{
		waitKey(); //macro: sets key or times out to waiting

		if(key == 11)
			break;
		else if(userid >= 100000000)
		{
			beepErr(1);
			goto WAITING;
		}
		beepKey();
		userid = userid*10 + key;
	}

	beepAck();
	userpin=0;

	//reading pin
	while(1)
	{
		waitKey(); //macro: sets key or times out to waiting

		if(key == 11)
			break;
		else
		{
			beepKey();
			userpin = userpin*10 + key;
		}
	}
	beepAck();
	sendEncryptedAuthRequest(userid, userpin);
	progmodeenterstate=1;

	//waiting for response
	while(authstate&AUTH_MASK_STATE == AUTH_WAIT)
	{
		spin();
		if (readKeypad())
			progmodeenterstate=0;

		if(centisecs-lastkeytime > 200)
		{
			lock();
			beepErr(3);
			restartEncryption();
			goto WAITING;
		}
	}
	if(authstate&AUTH_MASK_STATE != AUTH_READY)
	{
		lock();
		beepErr(4);
		restartEncryption();
		goto WAITING;
	}

	if(!authstate&AUTH_PERMISSION_AUTHENTICATED)
	{
		lock();
		beepErr(2);
		goto WAITING;
	}


	if(!authstate&AUTH_PERMISSION_ENTRY)
	{
		lock();
		beepUnlock();
		beepErr(2);
		goto WAITING;
	}

	lastkeytime = centisecs;
	unlock();
	beepUnlock();

	while (centisecs-lastkeytime < 500)
	{
		spin();
		key=readKeypad();
		if(key == 0 || progmodeenterstate == 0)
		{
			leds(0,0);
		}
		else if(progmodeenterstate == 1)
		{
			//enter held down as far as we know so far
			if (key==10)
				progmodeenterstate++;
			else
				progmodeenterstate=0;
		}
		else if(progmodeenterstate == 2)
		{
			//enter held down, zero pushed
			if (key==-10)
				progmodeenterstate++;
			else
				progmodeenterstate=0;
		}
		else if(progmodeenterstate < 10)
		{
			//enter held down, zero pushed and released
			if (key>0 && key < 10)
				progmodeenterstate=10+key;
			else if (key==10)
				progmodeenterstate=10;
			else 
				progmodeenterstate=0;
		}		
		else if(progmodeenterstate < 20)
		{
			//enter held down, zero pushed and released, other number pressed
			if (key==10-progmodeenterstate || key==-10 && progmodeenterstate==10)
				progmodeenterstate+=10;
			else 
				progmodeenterstate=0;
			
		}
		else if(progmodeenterstate < 30)
		{
			//enter held down, zero pushed and released, other number pressed and released
			if (key==-11)
			{
				progmodeenterstate-=20;
				beepAck();
				beepUnlock();
				leds(1,1);
				lastkeytime = centisecs;
				goto progmode;
			}
			else
				progmodeenterstate=0;
		}

	}

	lock();
	beepAck();

	goto WAITING;

progmode:
	switch(progmodeenterstate)
	{
		case 0://unlock (3hrs or until commanded by motion sensors)
		//if(authstate&AUTH_PERMISSION_ADMIN)
		case 1://relock
		case 2://password change
		case 3://reboot
		case 4://eeprom wipe
		case 5://add user
		case 6://del user
		waitKey(); //macro: sets key or times out to waiting
	}


	goto WAITING;

}




