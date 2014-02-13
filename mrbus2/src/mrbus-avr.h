/*************************************************************************
Title:    MRBus Atmel AVR Header
Authors:  Nathan Holmes <maverick@drgw.net>, Colorado, USA
          Michael Petersen <railfan@drgw.net>, Colorado USA
          Michael Prader, South Tyrol, Italy
File:     mrbus-avr.h
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan Holmes, Michael Petersen, and Michael Prader

    Original MRBus code developed by Nathan Holmes for PIC architecture.
    This file is based on AVR port by Michael Prader.  Updates and
    compatibility fixes by Michael Petersen.
    
    UART code derived from AVR UART library by Peter Fleury, and as
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

#ifndef MRBUS_AVR_H
#define MRBUS_AVR_H

// AVR type-specific stuff
// Define the UART port and registers used for XBee communication
// Follows the format of the AVR UART library by Fleury/Sharpe

#if defined(__AVR_ATmega162__)

#define MRBUS_ATMEGA_USART0_SIMPLE
#define MRBUS_UART_RX_INTERRUPT    USART0_RXC_vect
#define MRBUS_UART_TX_INTERRUPT    USART0_UDRE_vect
#define MRBUS_UART_DONE_INTERRUPT  USART0_TXC_vect
#define MRBUS_PORT                 PORTD
#define MRBUS_PIN                  PIND
#define MRBUS_DDR                  DDRD

#ifndef MRBUS_TXE
#define MRBUS_TXE                  2       /* PD2 */
#endif
#ifndef MRBUS_TX
#define MRBUS_TX                   1       /* PD1 */
#endif
#ifndef MRBUS_RX
#define MRBUS_RX                   0       /* PD0 */
#endif

#define MRBUS_UART_UBRR            UBRR0L
#define MRBUS_UART_SCR_A           UCSR0A
#define MRBUS_UART_SCR_B           UCSR0B
#define MRBUS_UART_SCR_C           UCSR0C
#define MRBUS_UART_DATA            UDR0
#define MRBUS_UART_UDRIE           UDRIE0
#define MRBUS_RXEN                 RXEN0
#define MRBUS_TXEN                 TXEN0
#define MRBUS_RXCIE                RXCIE0
#define MRBUS_TXCIE                TXCIE0
#define MRBUS_TXC                  TXC0
#define MRBUS_RX_ERR_MASK          (_BV(FE0) | _BV(DOR0))



#elif  defined(__AVR_ATmega8__)

#define MRBUS_ATMEGA_USART_SIMPLE
#define MRBUS_UART_RX_INTERRUPT    USART_RXC_vect
#define MRBUS_UART_TX_INTERRUPT    USART_UDRE_vect
#define MRBUS_UART_DONE_INTERRUPT  USART_TXC_vect
#define MRBUS_PORT                 PORTD
#define MRBUS_PIN                  PIND
#define MRBUS_DDR                  DDRD

#ifndef MRBUS_TXE
#define MRBUS_TXE                  2       /* PD2 */
#endif
#ifndef MRBUS_TX
#define MRBUS_TX                   1       /* PD1 */
#endif
#ifndef MRBUS_RX
#define MRBUS_RX                   0       /* PD0 */
#endif

#define MRBUS_UART_UBRR            UBRRL
#define MRBUS_UART_SCR_A           UCSRA
#define MRBUS_UART_SCR_B           UCSRB
#define MRBUS_UART_SCR_C           UCSRC
#define MRBUS_UART_DATA            UDR
#define MRBUS_UART_UDRIE           UDRIE
#define MRBUS_RXEN                 RXEN
#define MRBUS_TXEN                 TXEN
#define MRBUS_RXCIE                RXCIE
#define MRBUS_TXCIE                TXCIE
#define MRBUS_TXC                  TXC
#define MRBUS_RX_ERR_MASK          (_BV(FE) | _BV(DOR))



#elif defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega48P__) || defined(__AVR_ATmega88P__) || defined(__AVR_ATmega168P__) || \
    defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) 
#define MRBUS_ATMEGA_USART
#define MRBUS_UART_RX_INTERRUPT    USART_RX_vect
#define MRBUS_UART_TX_INTERRUPT    USART_UDRE_vect
#define MRBUS_UART_DONE_INTERRUPT  USART_TX_vect
#define MRBUS_PORT                 PORTD
#define MRBUS_PIN                  PIND
#define MRBUS_DDR                  DDRD

#ifndef MRBUS_TXE
#define MRBUS_TXE                  2       /* PD2 */
#endif
#ifndef MRBUS_TX
#define MRBUS_TX                   1       /* PD1 */
#endif
#ifndef MRBUS_RX
#define MRBUS_RX                   0       /* PD0 */
#endif

#define MRBUS_UART_UBRR            UBRR0
#define MRBUS_UART_SCR_A           UCSR0A
#define MRBUS_UART_SCR_B           UCSR0B
#define MRBUS_UART_SCR_C           UCSR0C
#define MRBUS_UART_DATA            UDR0
#define MRBUS_UART_UDRIE           UDRIE0
#define MRBUS_RXEN                 RXEN0
#define MRBUS_TXEN                 TXEN0
#define MRBUS_RXCIE                RXCIE0
#define MRBUS_TXCIE                TXCIE0
#define MRBUS_TXC                  TXC0
#define MRBUS_RX_ERR_MASK          (_BV(FE0) | _BV(DOR0))

#elif defined(__AVR_ATmega32U4__)

#define MRBUS_ATMEGA_USART1
#define MRBUS_UART_RX_INTERRUPT    USART1_RX_vect
#define MRBUS_UART_TX_INTERRUPT    USART1_UDRE_vect
#define MRBUS_UART_DONE_INTERRUPT  USART1_TX_vect
#define MRBUS_PORT                 PORTD
#define MRBUS_PIN                  PIND
#define MRBUS_DDR                  DDRD

#ifndef MRBUS_TXE
#define MRBUS_TXE                  2       /* PD2 */
#endif
#ifndef MRBUS_TX
#define MRBUS_TX                   1       /* PD1 */
#endif
#ifndef MRBUS_RX
#define MRBUS_RX                   0       /* PD0 */
#endif


#define MRBUS_UART_UBRR           UBRR1
#define MRBUS_UART_SCR_A          UCSR1A
#define MRBUS_UART_SCR_B          UCSR1B
#define MRBUS_UART_SCR_C          UCSR1C
#define MRBUS_UART_DATA           UDR1
#define MRBUS_UART_UDRIE          UDRIE1
#define MRBUS_RXEN                RXEN1
#define MRBUS_TXEN                TXEN1
#define MRBUS_RXCIE               RXCIE1
#define MRBUS_TXCIE               TXCIE1
#define MRBUS_TXC                 TXC1
#define MRBUS_RX_ERR_MASK         (_BV(FE1) | _BV(DOR1))


#elif defined(__AVR_ATmega164P__) || defined(__AVR_ATmega324P__) || \
    defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)

#define MRBUS_ATMEGA_USART0
#define MRBUS_UART_RX_INTERRUPT    USART0_RX_vect
#define MRBUS_UART_TX_INTERRUPT    USART0_UDRE_vect
#define MRBUS_UART_DONE_INTERRUPT  USART0_TX_vect
#define MRBUS_PORT                 PORTD
#define MRBUS_PIN                  PIND
#define MRBUS_DDR                  DDRD

#ifndef MRBUS_TXE
#define MRBUS_TXE                  4       /* PD4 */
#endif
#ifndef MRBUS_TX
#define MRBUS_TX                   1       /* PD1 */
#endif
#ifndef MRBUS_RX
#define MRBUS_RX                   0       /* PD0 */
#endif


#define MRBUS_UART_UBRR           UBRR0
#define MRBUS_UART_SCR_A          UCSR0A
#define MRBUS_UART_SCR_B          UCSR0B
#define MRBUS_UART_SCR_C          UCSR0C
#define MRBUS_UART_DATA           UDR0
#define MRBUS_UART_UDRIE          UDRIE0
#define MRBUS_RXEN                RXEN0
#define MRBUS_TXEN                TXEN0
#define MRBUS_RXCIE               RXCIE0
#define MRBUS_TXCIE               TXCIE0
#define MRBUS_TXC                 TXC0
#define MRBUS_RX_ERR_MASK         (_BV(FE0) | _BV(DOR0))

#elif defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__) || defined(__AVR_ATtiny4313__)

#define MRBUS_ATTINY_USART
#define MRBUS_UART_RX_INTERRUPT   USART_RX_vect
#define MRBUS_UART_TX_INTERRUPT   USART_UDRE_vect
#define MRBUS_UART_DONE_INTERRUPT USART_TX_vect
#define MRBUS_PORT                PORTD
#define MRBUS_PIN                 PIND
#define MRBUS_DDR                 DDRD

#ifndef MRBUS_TXE
#define MRBUS_TXE                 2       /* PD2 */
#endif
#ifndef MRBUS_TX
#define MRBUS_TX                  1       /* PD1 */
#endif
#ifndef MRBUS_RX
#define MRBUS_RX                  0       /* PD0 */
#endif

#define MRBUS_UART_UBRRH          UBRRH
#define MRBUS_UART_UBRRL          UBRRL
#define MRBUS_UART_SCR_A          UCSRA
#define MRBUS_UART_SCR_B          UCSRB
#define MRBUS_UART_SCR_C          UCSRC
#define MRBUS_UART_DATA           UDR
#define MRBUS_UART_UDRIE          UDRIE
#define MRBUS_RXEN                RXEN
#define MRBUS_TXEN                TXEN
#define MRBUS_RXCIE               RXCIE
#define MRBUS_TXCIE               TXCIE
#define MRBUS_TXC                 TXC
#define MRBUS_RX_ERR_MASK         (_BV(FE) | _BV(DOR))
#else
#error "No UART definition for MCU available"
#error "Please feel free to add one and send us the patch"
#endif

#endif // MRBUS_AVR_H


