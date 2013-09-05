/*************************************************************************
Title:    MRBus Microchip PIC Functions
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan Holmes <maverick@drgw.net>
File:     mrbus-pic.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan Holmes, Michael Petersen, and Michael Prader

    Original MRBus code developed by Nathan Holmes for PIC architecture.  
    This file is based on AVR port by Michael Prader.  Updates and 
    compatibility fixes by Michael Petersen.

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

#include <system.h>
#include "mrbus.h"

/* Variables used by MRBus code */
static uint8_t mrbus_rx_input_buffer[MRBUS_BUFFER_SIZE];
static uint8_t mrbus_rx_index;
static uint8_t mrbus_tx_index;
static uint8_t mrbus_loneliness;

/* Variables used by MRBus applications */
uint8_t mrbus_rx_buffer[MRBUS_BUFFER_SIZE];
uint8_t mrbus_tx_buffer[MRBUS_BUFFER_SIZE];
uint8_t mrbus_state;
uint8_t mrbus_priority;
uint8_t mrbus_activity;

uint8_t mrbusArbBitSend(uint8_t bitval)
{
	uint8_t slice, tmp=0;

	if (bitval)
		RS485PORT &= ~_BV(RS485TXEN);
	else
		RS485PORT |= _BV(RS485TXEN);

	for (slice=0; slice<10; slice++)
	{
		if (slice > 2)
		{
			if (UARTPORT & UART_RXMASK)
				tmp=1;

			if (tmp ^ bitval)
			{
				RS485PORT &= ~_BV(RS485TXEN);
				UARTTRIS |= _BV(TX);
				rcsta |= _BV(SPEN);
				return(1);
			}
			
		}
		delay_us(20);
	}

	return(0);
}

uint8_t mrbusPacketTransmit()
{
	uint8_t status;
	uint8_t address = mrbus_tx_buffer[MRBUS_PKT_SRC];
	uint8_t i;

	if (mrbus_state & MRBUS_TX_BUF_ACTIVE)
		return(1);


	mrbusCRC16Initialize();
	pie1 &= ~_BV(TXIE);

	/* First Calculate CRC16 */
	for(i=0; i<mrbus_tx_buffer[MRBUS_PKT_LEN]; i++)
	{
		if (i != MRBUS_PKT_CRC_L && i != MRBUS_PKT_CRC_H)
			mrbusCRC16Update(mrbus_tx_buffer[i]);
	}

	mrbus_tx_buffer[MRBUS_PKT_CRC_L] = crc16_low;
	mrbus_tx_buffer[MRBUS_PKT_CRC_H] = crc16_high;

	/* Start 2ms wait for activity */
	mrbus_activity=MRBUS_ACTIVITY_IDLE;

	asm clrwdt;
	delay_ms(2);
	
	/* Return if activity - we may have a packet to receive */
	/* Application is responsible for waiting 10ms or for successful receive */
	if (MRBUS_ACTIVITY_IDLE != mrbus_activity) 
	{
		if (mrbus_loneliness)
			mrbus_loneliness--;
		return(1);
	}	

	/* Now go into critical timing loop */
	/* Note that status is abused to calculate bus wait */
	status = ((mrbus_loneliness + mrbus_priority) * 10) + (address & 0x0F);

	RS485PORT &= ~_BV(RS485TXEN); //Clear driver enable to prevent transmitting */
	UARTPORT &= ~_BV(TX); // Set the TX pin low
	UARTTRIS &= ~_BV(TX); // Set the direction register to make TX an output
	rcsta &= ~_BV(SPEN);

	// Be sure to reset RX index - normally this is reset by other arbitration bytes causing
	//  framing errors, but if we're talking to ourselves, we're screwed because the RX
	//  side of the uart isn't on during arbitration sending
	mrbus_rx_index = 0;	

	for (i=0; i<44; i++)
	{
		delay_us(10);
		if (0 == UARTPORT & UART_RXMASK)
		{
			UARTTRIS |= _BV(TX);
			rcsta |= _BV(SPEN);
			if (mrbus_loneliness) 
				mrbus_loneliness--;
			return(1);
		}
	}

	/* Now, wait calculated time from above */
	for (i=0; i<status; i++)
	{
		delay_us(10);
		if (0 == UARTPORT & UART_RXMASK)
		{
			UARTTRIS |= _BV(TX);
			rcsta |= _BV(SPEN);
			if (mrbus_loneliness) 
				mrbus_loneliness--;
			return(1);
		}
	}

	/* Arbitration Sequence - 4800 bps */
	/* Start Bit */
	if (mrbusArbBitSend(0))
		return(1);

	for (i=0; i<8; i++)
	{
		status = mrbusArbBitSend(address & 0x01);
		address = address/2;

		if (status == 1)
			return(1);
	}	

	/* Stop Bits */
	if (mrbusArbBitSend(1)) return(1);
	if (mrbusArbBitSend(1)) return(1);

	// Go into active transmit mode
	
	UARTTRIS |= _BV(TX);  //Set TX back to input
	mrbus_rx_index = 0;
	mrbus_tx_index = 0;
	RS485PORT |= _BV(RS485TXEN);
	rcsta |= _BV(SPEN);
	txsta |= _BV(TXEN);

	
	while (intcon & _BV(GIE))
		intcon &= ~_BV(GIE);
	mrbus_state |= MRBUS_TX_BUF_ACTIVE;
	mrbus_state &= ~MRBUS_TX_PKT_READY;
	intcon |= _BV(GIE);
	pie1 |= _BV(TXIE);
	return(0);
}


void mrbusInit()
{
	mrbus_tx_index = 0;
	mrbus_rx_index = 0;
	mrbus_activity = MRBUS_ACTIVITY_IDLE;
	mrbus_loneliness = 6;
	mrbus_priority = 6;
	mrbus_state = 0;

	RS485TRIS &= ~_BV(RS485TXEN);
	RS485PORT &= ~_BV(RS485TXEN);
	UARTTRIS |= (_BV(RX) | _BV(TX));

	spbrg = 21;  /* 57.6kbps at 20MHz */
#ifdef TARGET_PIC16F88x
	spbrgh = 0;
	baudctl = 0; // _BV(BRG16);
#endif

	txsta = _BV(BRGH);
	rcsta = _BV(SPEN) | _BV(CREN);

	/* Enable receive interrupt */
	pie1 |= _BV(RCIE);
}

void mrbusPicTxISR()
{
	if ((pir1 & _BV(TXIF)) && (pie1 & _BV(TXIE)))
	{
		/* Transmit Routine */
		pir1 &= ~_BV(TXIF);
		if (mrbus_tx_index >= MRBUS_BUFFER_SIZE || mrbus_tx_index >= mrbus_tx_buffer[MRBUS_PKT_LEN])
		{
			while(!(txsta & _BV(TRMT)));
			pie1 &= ~_BV(TXIE);
			mrbus_state &= ~(MRBUS_TX_BUF_ACTIVE);
			RS485PORT &= ~_BV(RS485TXEN);
		} else {
			txreg = mrbus_tx_buffer[mrbus_tx_index++];
		}
	}
}

void mrbusPicRxISR()
{
	uint8_t bytenum;

	if ((pir1 & _BV(RCIF)) && (pie1 & _BV(RCIE)))
	{
		mrbus_activity = MRBUS_ACTIVITY_RX;
		if (rcsta & (_BV(FERR) | _BV(OERR)))
		{
			/* Handle framing errors - these are likely arbitration bytes */
			/* Overruns shouldn't happen */
			mrbus_rx_index = rcreg;
			rcsta &= ~_BV(CREN);
			mrbus_rx_index = 0; /* Reset receive buffer */
			rcsta |= _BV(CREN);
		} else {
			/* Receive Routine */
			mrbus_rx_input_buffer[mrbus_rx_index++] = rcreg;
	
			if ((mrbus_rx_index > 4) && ((mrbus_rx_index == mrbus_rx_input_buffer[MRBUS_PKT_LEN]) || (mrbus_rx_index >=  MRBUS_BUFFER_SIZE) ))
			{
				mrbus_rx_index = 0;
	
				if((mrbus_state & MRBUS_RX_PKT_READY) == 0)
				{
					if (mrbus_rx_input_buffer[MRBUS_PKT_LEN] > MRBUS_BUFFER_SIZE)
						mrbus_rx_input_buffer[MRBUS_PKT_LEN] = MRBUS_BUFFER_SIZE;
					
					for(bytenum=0; bytenum < mrbus_rx_input_buffer[MRBUS_PKT_LEN]; bytenum++)
					{
						mrbus_rx_buffer[bytenum] = mrbus_rx_input_buffer[bytenum];
					}
					mrbus_state |= MRBUS_RX_PKT_READY;
					mrbus_activity = MRBUS_ACTIVITY_RX_COMPLETE;
				}
			}
			if (mrbus_rx_index >= MRBUS_BUFFER_SIZE)
				mrbus_rx_index = MRBUS_BUFFER_SIZE-1;
		}
	}
}


