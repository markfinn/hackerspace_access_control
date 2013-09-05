/*************************************************************************
Title:    MRBus Microchip PIC Header
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan Holmes <maverick@drgw.net>
File:     mrbus-pic.h
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan Holmes, Michael Petersen, and Michael Prader

    Original code developed by Nathan Holmes for PIC architecture.  Based
    on AVR port by Michael Prader.  Updates and compatibility fixes by
    Michael Petersen.
    
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
#ifndef MRBUS_PIC_H
#define MRBUS_PIC_H

#ifndef _BV
#define _BV(a) (1<<(a))
#endif

#define RCIF_MASK _BV(RCIF)
#define RCIE_MASK _BV(RCIE)
#define TXIF_MASK _BV(TXIF)
#define TXIE_MASK _BV(TXIE)

#define RC_ERR_MASK  (_BV(FERR) | _BV(OERR))
#define FERR_MASK _BV(FERR)
#define OERR_MASK _BV(OERR)
#define TRMT_MASK _BV(TRMT)
#define T0IE_MASK _BV(T0IE)
#define T0IF_MASK _BV(T0IF)
#define INTE_MASK _BV(INTE)
#define INTF_MASK _BV(INTF)
#define BF_MASK   0x01

#define SDO  5
#define SDI  4
#define SCK  3

/* PIC Specific Stuff */
#ifdef TARGET_PIC16F62x
#define TX   2
#define RX   1
#define RS485TXEN 3
#define UART_RXMASK 0x02

#define UARTPORT portb
#define UARTTRIS trisb
#define RS485PORT portb
#define RS485TRIS trisb
#endif

#ifdef TARGET_PIC16F87x
#define TX   6
#define RX   7
#define RS485TXEN 2
#define UART_RXMASK 0x80

#define UARTPORT portc
#define UARTTRIS trisc
#define RS485PORT portc
#define RS485TRIS trisc

#endif

#ifdef TARGET_PIC16F88x

#define TX   6
#define RX   7
#define RS485TXEN 2
#define UART_RXMASK 0x80

#define UARTPORT portc
#define UARTTRIS trisc

#define RS485PORT portc
#define RS485TRIS trisc

#endif


/* Device specific defines */
#define T0_Preload 60
#pragma CLOCK_FREQ 20000000

void mrbusPicRxISR();
void mrbusPicTxISR();

#endif
