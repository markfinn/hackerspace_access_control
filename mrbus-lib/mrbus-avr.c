/*************************************************************************
Title:    MRBus Atmel AVR Functions
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan Holmes <maverick@drgw.net>
File:     mrbus-avr.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan Holmes, Michael Petersen, and Michael Prader

    Original MRBus code developed by Nathan Holmes for PIC architecture.
    This file is based on AVR port by Michael Prader.  Updates and
    compatibility fixes by Michael Petersen.
    
    UART code inspired by from AVR UART library by Peter Fleury, and as
    modified by Tim Sharpe.

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


#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "mrbus.h"

/* Variables used by MRBus code */
static volatile uint8_t mrbus_rx_input_buffer[MRBUS_BUFFER_SIZE];
static volatile uint8_t mrbus_rx_index;
static volatile uint8_t mrbus_tx_index;
static uint8_t mrbus_loneliness;

/* Variables used by MRBus applications */
volatile uint8_t mrbus_rx_buffer[MRBUS_BUFFER_SIZE];
volatile uint8_t mrbus_tx_buffer[MRBUS_BUFFER_SIZE];
volatile uint8_t mrbus_state;
volatile uint8_t mrbus_priority;
volatile uint8_t mrbus_activity;


uint8_t mrbusArbBitSend(uint8_t bitval)
{
	uint8_t slice;
	uint8_t tmp = 0;

	cli();
	if (bitval)
	{
		MRBUS_PORT &= ~_BV(MRBUS_TXE);
	}
	else
	{
		MRBUS_PORT |= _BV(MRBUS_TXE);
	}

	for (slice = 0; slice < 10; slice++)
	{
		if (slice > 2)
		{
			if (MRBUS_PIN & _BV(MRBUS_RX)) tmp = 1;
			if (tmp ^ bitval)
			{
				MRBUS_PORT &= ~_BV(MRBUS_TXE);
				MRBUS_DDR &= ~_BV(MRBUS_TX);
				return(1);
			}

		}
		_delay_us(20);
	}
	return(0);
}


uint8_t mrbusPacketTransmit(void)
{
	uint8_t status;
	uint8_t address;
	uint8_t i;
	uint16_t crc16_value = 0x0000;

	//  Return if bus already active.
	//  However, pending packet may already be trashed by application re-writing the buffer
	if (mrbus_state & MRBUS_TX_BUF_ACTIVE)
		return(1);

	// If we have no packet length, or it's less than the header, just silently say we transmitted it
	// On the AVRs, if you don't have any packet length, it'll never clear up on the interrupt routine
	// and you'll get stuck in indefinite transmit busy
	if (mrbus_tx_buffer[MRBUS_PKT_LEN] < MRBUS_PKT_TYPE)
	{
		mrbus_state &= ~(MRBUS_TX_BUF_ACTIVE | MRBUS_TX_PKT_READY);
		return(0);
	}
		
		
	address = mrbus_tx_buffer[MRBUS_PKT_SRC];

	// First Calculate CRC16
	for (i = 0; i < mrbus_tx_buffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L))
		{
			crc16_value = mrbusCRC16Update(crc16_value, mrbus_tx_buffer[i]);
		}
	}
	mrbus_tx_buffer[MRBUS_PKT_CRC_L] = (crc16_value & 0xFF);
	mrbus_tx_buffer[MRBUS_PKT_CRC_H] = ((crc16_value >> 8) & 0xFF);

	/* Start 2ms wait for activity */
	mrbus_activity = MRBUS_ACTIVITY_IDLE;
	_delay_ms(2);

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

	MRBUS_PORT &= ~_BV(MRBUS_TXE);  /* Clear driver enable to prevent transmitting */
	MRBUS_PORT &= ~_BV(MRBUS_TX);   /* Set the XMIT pin low */
	MRBUS_DDR |= _BV(MRBUS_TX);     /* Set as output */

	//  Disable transmitter
	cli();
	MRBUS_UART_SCR_B &= ~_BV(MRBUS_TXEN);
	sei();
	
	// Be sure to reset RX index - normally this is reset by other arbitration bytes causing
	//  framing errors, but if we're talking to ourselves, we're screwed because the RX
	//  side of the uart isn't on during arbitration sending
	mrbus_rx_index = 0;

	for (i = 0; i < 44; i++)
	{
		_delay_us(10);
		if (0 == (MRBUS_PIN & _BV(MRBUS_RX)))
		{
			MRBUS_DDR &= ~_BV(MRBUS_TX);
			if (mrbus_loneliness)
				mrbus_loneliness--;
			return(1);
		}
	}

	/* Now, wait calculated time from above */
	for (i = 0; i < status; i++)
	{
		_delay_us(10);
		if (0 == (MRBUS_PIN & _BV(MRBUS_RX)))
		{
			MRBUS_DDR &= ~_BV(MRBUS_TX);
			if (mrbus_loneliness)
				mrbus_loneliness--;
			return(1);
		}
	}

	/* Arbitration Sequence - 4800 bps */
	/* Start Bit */
	if (mrbusArbBitSend(0))
	{
		if (mrbus_loneliness)
			mrbus_loneliness--;
		return(1);
	}

	for (i = 0; i < 8; i++)
	{
		status = mrbusArbBitSend(address & 0x01);
		address = address / 2;

		if (status)
		{
			if (mrbus_loneliness) mrbus_loneliness--;
			return(1);
		}
	}

	/* Stop Bits */
	if (mrbusArbBitSend(1))
	{
		if (mrbus_loneliness) mrbus_loneliness--;
		return(1);
	}
	if (mrbusArbBitSend(1))
	{
		if (mrbus_loneliness) mrbus_loneliness--;
		return(1);
	}

	/* Set TX back to input */
	MRBUS_DDR &= ~_BV(MRBUS_TX);
	/* Enable transmitter since control over bus is assumed */
	MRBUS_UART_SCR_B |= _BV(MRBUS_TXEN);
	MRBUS_PORT |= _BV(MRBUS_TXE);

	mrbus_tx_index = 0;

	mrbus_state |= MRBUS_TX_BUF_ACTIVE;
	mrbus_state &= ~MRBUS_TX_PKT_READY;
	
	cli();
#ifdef MRBUS_DISABLE_LOOPBACK
	// Disable receive interrupt while transmitting
	MRBUS_UART_SCR_B &= ~_BV(MRBUS_RXCIE);
#endif
	// Enable transmit interrupt
	MRBUS_UART_SCR_B |= _BV(MRBUS_UART_UDRIE);
	sei();

	return(0);
}


void mrbusInit(void)
{
	MRBUS_DDR |= _BV(MRBUS_TXE);
	MRBUS_PORT &= ~_BV(MRBUS_TXE);

	MRBUS_DDR &= ~_BV(MRBUS_RX);
	MRBUS_DDR &= ~_BV(MRBUS_TX);

	mrbus_tx_index = 0;
	mrbus_rx_index = 0;
	mrbus_activity = MRBUS_ACTIVITY_IDLE;
	mrbus_loneliness = 6;
	mrbus_priority = 6;
	mrbus_state = 0;

#undef BAUD
#define BAUD MRBUS_BAUD
#include <util/setbaud.h>

#if defined( MRBUS_AT90_UART )
	// FIXME - probably need more stuff here
	UBRR = (uint8_t)UBRRL_VALUE;

#elif defined( MRBUS_ATMEGA_USART_SIMPLE )
    MRBUS_UART_UBRR = UBRR_VALUE;
	MRBUS_UART_SCR_A = (USE_2X)?_BV(U2X):0;
	MRBUS_UART_SCR_B = 0;
	MRBUS_UART_SCR_C = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);
	
#elif defined( MRBUS_ATMEGA_USART0_SIMPLE )
    MRBUS_UART_UBRR = UBRR_VALUE;
	MRBUS_UART_SCR_A = (USE_2X)?_BV(U2X0):0;
	MRBUS_UART_SCR_B = 0;
	MRBUS_UART_SCR_C = _BV(URSEL0) | _BV(UCSZ01) | _BV(UCSZ00);
	
#elif defined( MRBUS_ATMEGA_USART ) || defined ( MRBUS_ATMEGA_USART0 )
	MRBUS_UART_UBRR = UBRR_VALUE;
	MRBUS_UART_SCR_A = (USE_2X)?_BV(U2X0):0;
	MRBUS_UART_SCR_B = 0;
	MRBUS_UART_SCR_C = _BV(UCSZ01) | _BV(UCSZ00);

#elif defined( MRBUS_ATTINY_USART )
	// Top four bits are reserved and must always be zero - see ATtiny2313 datasheet
	// Also, H and L must be written independently, since they're non-adjacent registers
	// on the attiny parts
	MRBUS_UART_UBRRH = 0x0F & UBRRH_VALUE;
	MRBUS_UART_UBRRL = UBRRL_VALUE;
	MRBUS_UART_SCR_A = (USE_2X)?_BV(U2X):0;
	MRBUS_UART_SCR_B = 0;
	MRBUS_UART_SCR_C = _BV(UCSZ1) | _BV(UCSZ0);

#elif defined ( MRBUS_ATMEGA_USART1 )
	MRBUS_UART_UBRR = UBRR_VALUE;
	MRBUS_UART_SCR_A = (USE_2X)?_BV(U2X1):0;
	MRBUS_UART_SCR_B = 0;
	MRBUS_UART_SCR_C = _BV(UCSZ11) | _BV(UCSZ10);
#endif

#undef BAUD

	/* Enable USART receiver and transmitter and receive complete interrupt */
	MRBUS_UART_SCR_B = _BV(MRBUS_RXCIE) | _BV(MRBUS_RXEN) | _BV(MRBUS_TXEN);

}

ISR(MRBUS_UART_DONE_INTERRUPT)
{
	// Transmit is complete: terminate
	MRBUS_PORT &= ~_BV(MRBUS_TXE);
	MRBUS_UART_SCR_B &= ~(_BV(MRBUS_TXCIE) | _BV(MRBUS_TXEN) );
	// Re-enable receive interrupt
	MRBUS_UART_SCR_B |= _BV(MRBUS_RXCIE);
	mrbus_state &= ~MRBUS_TX_BUF_ACTIVE;
	mrbus_loneliness = 6;
}

ISR(MRBUS_UART_TX_INTERRUPT)
{
	// Transmit Routine
	if (mrbus_tx_index == 1)
	{
		// Set transmit complete interrupt after packet in underway
		MRBUS_UART_SCR_B |= _BV(MRBUS_TXCIE);
	}

	if ((mrbus_tx_index >= MRBUS_BUFFER_SIZE) || (mrbus_tx_index >= mrbus_tx_buffer[MRBUS_PKT_LEN]))
	{

		// Turn off transmit buffer interrupt
		// Wait for transmit complete interrupt to finish cleanup
		MRBUS_UART_SCR_B &= ~_BV(MRBUS_UART_UDRIE);
	}
	else
	{
		MRBUS_UART_DATA = mrbus_tx_buffer[mrbus_tx_index++];
	}
}


ISR(MRBUS_UART_RX_INTERRUPT)
{
	//Receive Routine
	mrbus_activity = MRBUS_ACTIVITY_RX;

	if (MRBUS_UART_SCR_A & MRBUS_RX_ERR_MASK)
	{
		// Handle framing errors - these are likely arbitration bytes
		mrbus_rx_index = MRBUS_UART_DATA;
		mrbus_rx_index = 0; // Reset receive buffer
	}
	else
	{
		mrbus_rx_input_buffer[mrbus_rx_index++] = MRBUS_UART_DATA;
		if ((mrbus_rx_index > 5) && ((mrbus_rx_index >= mrbus_rx_input_buffer[MRBUS_PKT_LEN]) || (mrbus_rx_index >= MRBUS_BUFFER_SIZE) ))
		{
			uint8_t i;
			mrbus_rx_index = 0;
			if (!(mrbus_state & MRBUS_RX_PKT_READY))
			{
				if (mrbus_rx_input_buffer[MRBUS_PKT_LEN] > MRBUS_BUFFER_SIZE) mrbus_rx_input_buffer[MRBUS_PKT_LEN] = MRBUS_BUFFER_SIZE;

				for (i = 0; i < mrbus_rx_input_buffer[MRBUS_PKT_LEN]; i++)
				{
					mrbus_rx_buffer[i] = mrbus_rx_input_buffer[i];
				}
				mrbus_state = mrbus_state | MRBUS_RX_PKT_READY;
				mrbus_activity = MRBUS_ACTIVITY_RX_COMPLETE;
			}
		}
		if (mrbus_rx_index >= MRBUS_BUFFER_SIZE)
		{
			mrbus_rx_index = MRBUS_BUFFER_SIZE - 1;
		}
	}
}
