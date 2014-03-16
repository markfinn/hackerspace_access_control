/*************************************************************************
Title:    MRBus Computer Interface v2
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3


LICENSE:
    Copyright (C) 2014 Nathan Holmes
    
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
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <ctype.h>
#include <string.h>
#include "mrbus.h"


#define RING_BUFFER_SZ  128
#include "avr-ringbuffer.h"


#define min(a,b)  ((a)<(b)?(a):(b))


#define UART1_BAUD 115200
#define FLOW_BUFFER_FULL (RING_BUFFER_SZ - 8)
#define FLOW_BUFFER_REENABLE (RING_BUFFER_SZ - 32)
#define CTS_OUT_PIN  PD6
#define RTS_IN_PIN   PD5

#define MRBUS_RX_BUFFER_DEPTH 8
#define MRBUS_TX_BUFFER_DEPTH 4


inline void flowStopReceive()
{
	PORTD |= _BV(CTS_OUT_PIN);
}


inline void flowStartReceive()
{
	PORTD &= ~_BV(CTS_OUT_PIN);
}


#define APPSTATE_SHUT_OFF_PKTS  0x01
#define APPSTATE_CMD_READY      0x02
#define APPSTATE_NO_STALL   		0x40
#define APPSTATE_MACHINE_MODE   0x80


RingBuffer serialTXBuffer;
RingBuffer serialRXBuffer;
MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];
uint8_t applicationState = 0;






volatile uint8_t timeout = 0;

void txBufferPush(uint8_t data)
{
	uint8_t sent=0;
	while(!sent)
		sent = ringBufferPushNonBlocking(&serialTXBuffer, data);

	// If RTS is high, do not start TX ISR
	if (PIND & _BV(RTS_IN_PIN))
		return;


	UCSR1B |= _BV(UDRIE1);
}


void txBufferPushString(char* str)
{
	while (*str != 0)
		txBufferPush(*str++);
}

void txBufferPushString_P(const char* str)
{
	uint8_t c;
	while (0 != (c = pgm_read_byte(str++)))
		txBufferPush(c);
}




ISR(USART1_UDRE_vect)
{
	if (PIND & _BV(RTS_IN_PIN))
	{
		UCSR1B &= ~_BV(UDRIE1);
		return;
	}


	if (ringBufferDepth(&serialTXBuffer))
		UDR1 = ringBufferPopNonBlocking(&serialTXBuffer);
	else
		UCSR1B &= ~_BV(UDRIE1);
}


ISR(USART1_RX_vect)
{
	uint8_t status = UCSR1A, data = UDR1;


	// Framing errors and other crap.  Throw it out
	if (status & (_BV(FE1) | _BV(DOR1) | _BV(UPE1) ))
		return;
	
	ringBufferPushNonBlocking(&serialRXBuffer, data);

	// If we're less than 8 from filling buffer, raise CTS
	if (ringBufferDepth(&serialRXBuffer) > FLOW_BUFFER_FULL)
		flowStopReceive();


	return;
}


void terminalUartInitialize()
{
	//UCSR1A
#define BAUD UART1_BAUD
#include <util/setbaud.h>


	UBRR1 = UBRR_VALUE;
	UCSR1A = (USE_2X)?_BV(U2X1):0;
	UCSR1B = _BV(RXEN1) | _BV(TXEN1) | _BV(RXCIE1);
	UCSR1C = _BV(UCSZ11) | _BV(UCSZ10);
#undef BAUD
}


volatile uint16_t ticks50khz=0;


void initialize100HzTimer(void)
{
	// Set up timer 1 for 50kHz (20uS) interrupts
	TCNT0 = 0;
	OCR0A = 50;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS01);// div/8 prescaler
	TIMSK0 |= _BV(OCIE0A);
}


ISR(TIMER0_COMPA_vect)
{
	ticks50khz++;

	//check if flow cntrol should be unquenched.  This is also done in the main loop because of a deadlock.  I think I fixed that deadlock, but don't want to try removing this just yet.
	if (0 == (PIND & _BV(RTS_IN_PIN)) 
		&& 0 == (UCSR1B & _BV(UDRIE1))
		&& ringBufferDepth(&serialTXBuffer))
	{
		UCSR1B |= _BV(UDRIE1);
	}
}


// End of 100Hz timer


volatile uint8_t busVoltage=0;

ISR(ADC_vect)
{
	static uint16_t busVoltageAccum=0;
	static uint8_t busVoltageCount=0;

	busVoltageAccum += ADC;
	if (++busVoltageCount >= 64)
	{
		busVoltageAccum = busVoltageAccum / 64;
        //At this point, we're at (Vbus/3) / 5 * 1024
        //So multiply by 300, divide by 1024, or multiply by 150 and divide by 512
        busVoltage = ((uint32_t)busVoltageAccum * 75) / 512;
		busVoltageAccum = 0;
		busVoltageCount = 0;
	}
}

void initializeADC()
{
	// Setup ADC
	ADMUX  = 0x40;  // AVCC reference; ADC7 input
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x01;  // Turn off PA0, as it's really ADC0 now
	DIDR1  = 0x00;
	busVoltage = 0;
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
}



void init(void)
{
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


	initialize100HzTimer();

	// PA0 is the bus voltage analog input
	// PA7 is the random unassigned jumper
	DDRA &= ~(_BV(PA0) | _BV(PA7));
	PORTA &= ~_BV(PA0);
	// Turn on pullups for jumper
	PORTA = _BV(PA7);
	
	// Set CTS to an output, RTS to an input, and  
	DDRD |= _BV(CTS_OUT_PIN);
	DDRD &= ~_BV(RTS_IN_PIN);
	flowStopReceive(); // Stop receiving we've actually gotten ISRs up and running

	initializeADC();
	
	// Initialize both MRB-CI2 <==> FT232 buffers
	terminalUartInitialize();
	ringBufferInitialize(&serialTXBuffer);
	ringBufferInitialize(&serialRXBuffer);

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);	
	mrbusInit();

	sei();
	flowStartReceive();	
}


const uint8_t NibbleToHex[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

// The text representation is 2 for P:, 3 per hex byte, 2 for \r\n, and 1 for good luck
#define MRBUS_TEXT_PKT_SIZE(a) (2 + (a)*3 + 2 + 1)



void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t crcBogus = 0, i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	
	if (mrbusPktQueueEmpty(&mrbusRxQueue) || (applicationState & APPSTATE_SHUT_OFF_PKTS))
		return;

	// Since we've already tested for empty, guaranteed to get us a packet
	// Also, we're just peeking it so that we can get at it, look, and see if we have enough room in the TX queue to buffer it up
	mrbusPktQueuePeek(&mrbusRxQueue, rxBuffer, sizeof(rxBuffer));

	// Not enough room to shove it in the output buffer, pop out	
	if ((RING_BUFFER_SZ - ringBufferDepth(&serialTXBuffer)) < MRBUS_TEXT_PKT_SIZE(rxBuffer[MRBUS_PKT_LEN]))
		return;
		
	// Okay, we've confirmed enough space, clear up our slot in the RX queue
	mrbusPktQueueDrop(&mrbusRxQueue);
	
	/*************** PACKET FILTER ***************/
	/* CRC16 Test - is the packet intact? */
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		switch(i)
		{
			case MRBUS_PKT_CRC_H:
			case MRBUS_PKT_CRC_L:
				break; // Do nothing, CRC not included in CRC for obvious reasons

			default:
				crc = mrbusCRC16Update(crc, rxBuffer[i]);
				break;
		}
	}

	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
		crcBogus = 1;

	/*************** END PACKET FILTER ***************/
	
	/*************** PACKET SUCCESS - PROCESS HERE ***************/
	
	txBufferPush(crcBogus?'E':'P');
	txBufferPush(':');
	
	for (i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		txBufferPush(NibbleToHex[((rxBuffer[i]>>4) & 0x0F)]);
		txBufferPush(NibbleToHex[(rxBuffer[i] & 0x0F)]);
		txBufferPush(' ');
	}
	txBufferPushString_P(PSTR("\r\n"));
}


#define TERMINAL_WAITING_FOR_START  0
#define TERMINAL_COLLECTING_DATA    1
#define TERMINAL_INPUT_COMPLETE     2


uint8_t HexToBinary(uint8_t h, uint8_t l)
{
	uint8_t retval = 0;
	h = toupper(h);
	l = toupper(l);
	
	if(isdigit(h))
		retval = h-'0';
	else if (h >= 'A' && h <= 'F')
		retval = h - 'A' + 10;
	
	retval *= 16;


	if(isdigit(l))
		retval += l - '0';
	else if (l >= 'A' && l <= 'F')
		retval += l - 'A' + 10;


	return(retval);
}


int main(void)
{
	uint8_t pktIncomingState = TERMINAL_WAITING_FOR_START, data;
	uint8_t terminalCommandBuffer[64];
	uint8_t terminalCommandBufferIdx = 0;
	uint8_t i;


	init();

	txBufferPushString_P(PSTR("I:Reset\r\n"));
	while (1)
	{
		wdt_reset();
		
/*		if (decisecs >= 10)
		{
			decisecs = 0;
			txBufferPushString("I:MRX=");
			i = mrbusPktQueueDepth(&mrbusRxQueue);
			txBufferPush(NibbleToHex[(i>>4) & 0x0F]);
			txBufferPush(NibbleToHex[i & 0x0F]);
			txBufferPushString("\r\n");
		}
	*/	


		if (0 == (PIND & _BV(RTS_IN_PIN)) 
			&& 0 == (UCSR1B & _BV(UDRIE1))
			&& ringBufferDepth(&serialTXBuffer))
		{
			UCSR1B |= _BV(UDRIE1);
		}


		while(ringBufferDepth(&serialRXBuffer) && !mrbusPktQueueFull(&mrbusTxQueue))
		{
			data = ringBufferPopNonBlocking(&serialRXBuffer);


			// If we've got 12 or more bytes, drop CTS to allow more RX
			if (ringBufferDepth(&serialRXBuffer) < FLOW_BUFFER_REENABLE)
				flowStartReceive();


			if (TERMINAL_WAITING_FOR_START == pktIncomingState && ':' != data)
			{
				memset(terminalCommandBuffer, 0, sizeof(terminalCommandBuffer));
				terminalCommandBufferIdx = 0;
				if (0x0D == data)
				{
					txBufferPushString_P(PSTR("\r\nCmd Error\r\n"));
				}
			}
			else if (':' == data)
			{
				pktIncomingState = TERMINAL_COLLECTING_DATA;
				memset(terminalCommandBuffer, 0, sizeof(terminalCommandBuffer));
				terminalCommandBufferIdx = 0;
				terminalCommandBuffer[terminalCommandBufferIdx++] = data;
				// Start of incoming command
				if (!(APPSTATE_MACHINE_MODE & applicationState))
					applicationState |= APPSTATE_SHUT_OFF_PKTS;
			}
			else if (TERMINAL_COLLECTING_DATA == pktIncomingState && 0x0D != data)
			{
				if (0x08 == data) // Backspace
				{
					if (terminalCommandBufferIdx)
						terminalCommandBufferIdx--;
				}
				else
				{
					terminalCommandBuffer[terminalCommandBufferIdx++] = data;
					terminalCommandBufferIdx = min(sizeof(terminalCommandBuffer), terminalCommandBufferIdx);
				}
			}
			else if (0x0D == data)
			{
				uint8_t badCmd = 0;
				// Potential end of sequence
				if (terminalCommandBuffer[0] != ':')
					badCmd = 1;
				if (terminalCommandBuffer[(terminalCommandBufferIdx-1)] != ';')
					badCmd = 2;


				// Commands are in the format of...
				// :CMD TT VV;
				//  TT = type of command, two characters
				//  VV = value to set


				if (0 == badCmd && (terminalCommandBufferIdx > 5) && 0 == memcmp(terminalCommandBuffer+1, "CMD ", 4))
				{
					uint8_t value = HexToBinary(terminalCommandBuffer[8], terminalCommandBuffer[9]);
					badCmd = 0;
					
					// it's a command packet, not a bus packet
					if (0 == memcmp(terminalCommandBuffer+5, "MM", 2))
					{
						if (value)
							applicationState |= APPSTATE_MACHINE_MODE;
						else
							applicationState &= ~(APPSTATE_MACHINE_MODE);
					}
					else if (0 == memcmp(terminalCommandBuffer+5, "NS", 2))
					{
						if (value)
							applicationState |= APPSTATE_NO_STALL;
						else
							applicationState &= ~(APPSTATE_NO_STALL);
					}
					else if (0 == memcmp(terminalCommandBuffer+5, "BV", 2))
					{
						value = busVoltage;
					}
					else
					{
						badCmd = 4;
					}

					if (badCmd)
						txBufferPushString_P(PSTR("\r\nUnknown Cmd\r\n"));
					else
					{
						txBufferPushString_P(PSTR("\r\nOk"));
						txBufferPushString_P(PSTR("\r\nI:"));
						txBufferPush(terminalCommandBuffer[5]);
						txBufferPush(terminalCommandBuffer[6]);
						txBufferPush('=');
						txBufferPush(NibbleToHex[(value>>4) & 0x0F]);
						txBufferPush(NibbleToHex[value & 0x0F]);
						txBufferPushString_P(PSTR("\r\n"));						
					}

				}
				else
				{
					// It's a bus packet
					if ('-' != terminalCommandBuffer[3] || '>' != terminalCommandBuffer[4]) 
						badCmd = 3;
	
					if (!(APPSTATE_MACHINE_MODE & applicationState))
						txBufferPushString("\r\n");


					if (badCmd)
					{
						txBufferPushString_P(PSTR("Cmd Error ["));
						txBufferPush(badCmd + '0');
						txBufferPushString_P(PSTR("]\r\n"));
					}
					else
					{
						uint8_t mrbTxIdx = 5, cmdBufferIdx = 8;

						// Spin on mrbus pkts still in the buffer for a while
						if(!(APPSTATE_NO_STALL & applicationState))
							while(mrbusPktQueueFull(&mrbusTxQueue))
							{
								// Handle any packets that may have come in
								if (mrbusPktQueueDepth(&mrbusRxQueue))
									PktHandler();							
								if (mrbusPktQueueDepth(&mrbusTxQueue))
									mrbusTransmit();
							}
					
						if (mrbusPktQueueFull(&mrbusTxQueue))
							txBufferPushString_P(PSTR("Overflow\r\n"));
						else
						{
							uint8_t txBuffer[MRBUS_BUFFER_SIZE];
							memset(txBuffer, 0, sizeof(txBuffer));
							txBuffer[MRBUS_PKT_SRC] = HexToBinary(terminalCommandBuffer[1], terminalCommandBuffer[2]);
							txBuffer[MRBUS_PKT_DEST] = HexToBinary(terminalCommandBuffer[5], terminalCommandBuffer[6]);
							while(terminalCommandBuffer[cmdBufferIdx] != 0 && cmdBufferIdx<sizeof(terminalCommandBuffer))
							{
								if (mrbTxIdx > 19)
								{
									badCmd = 5;
									break;
								}
								txBuffer[mrbTxIdx] = HexToBinary(terminalCommandBuffer[cmdBufferIdx], terminalCommandBuffer[cmdBufferIdx+1]);
								cmdBufferIdx += 3;
								mrbTxIdx++;
							}
							txBuffer[MRBUS_PKT_LEN] = mrbTxIdx;


							if (badCmd)
								txBufferPushString_P(PSTR("Oversize pkt\r\n"));
							else
							{	
								mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
								txBufferPushString_P(PSTR("Ok\r\n"));
							}
						}
					}
				}
				applicationState &= ~(APPSTATE_SHUT_OFF_PKTS);
				pktIncomingState = TERMINAL_WAITING_FOR_START;
			}
			
			
			// In human mode (aka, non-machine), echo characters back to the terminal
			if (!(APPSTATE_MACHINE_MODE & applicationState) && !(pktIncomingState == TERMINAL_WAITING_FOR_START))
				txBufferPush(data);
		}
	
		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
			PktHandler();	

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
				uint8_t bus_countdown = 20;
				while (bus_countdown-- > 0 && !mrbusIsBusIdle())
				{
					wdt_reset();
					// If we've got space, drop CTS to allow more RX
					if (ringBufferDepth(&serialRXBuffer) < FLOW_BUFFER_REENABLE)
						flowStartReceive();
					// Process anything that came in
					if (mrbusPktQueueDepth(&mrbusRxQueue))
						PktHandler();

					_delay_ms(1);
				}
			}
		}
	}
}




//:ff->ff 41;

