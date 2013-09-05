/*************************************************************************
Title:    MRBus Common Header File
Authors:  Nathan Holmes <maverick@drgw.net>, Colorado, USA
          Michael Petersen <railfan@drgw.net>, Colorado USA
          Michael Prader, South Tyrol, Italy
File:     mrbus.h
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan Holmes, Michael Petersen, and Michael Prader

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

#ifndef MRBUS_H
#define MRBUS_H

#ifdef __AVR__
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#endif

#ifdef _PIC16
#include <system.h>
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
#endif

// Size definitions
#define MRBUS_BUFFER_SIZE  0x14

// Packet component defines
#define MRBUS_PKT_DEST  0
#define MRBUS_PKT_SRC   1
#define MRBUS_PKT_LEN   2
#define MRBUS_PKT_CRC_L 3
#define MRBUS_PKT_CRC_H 4
#define MRBUS_PKT_TYPE  5
#define MRBUS_PKT_SUBTYPE 6

// mrbus_status masks
#define MRBUS_RX_PKT_READY  0x01
#define MRBUS_TX_BUF_ACTIVE 0x40
#define MRBUS_TX_PKT_READY  0x80

// mrbus_activity states
#define MRBUS_ACTIVITY_IDLE          0
#define MRBUS_ACTIVITY_RX            1
#define MRBUS_ACTIVITY_RX_COMPLETE   2

// Specification-defined EEPROM Addresses
#define MRBUS_EE_DEVICE_ADDR         0
#define MRBUS_EE_DEVICE_OPT_FLAGS    1
#define MRBUS_EE_DEVICE_UPDATE_H     2
#define MRBUS_EE_DEVICE_UPDATE_L     3

// Version flags
#define MRBUS_VERSION_WIRELESS 0x80
#define MRBUS_VERSION_WIRED    0x00

#define MRBUS_BAUD   57600

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __AVR__
#include "mrbus-avr.h"
uint16_t mrbusCRC16Update(uint16_t crc, uint8_t a);
#endif

#ifdef _PIC16
#include "mrbus-pic.h"
extern uint8_t crc16_high;
extern uint8_t crc16_low;
void mrbusCRC16Update(uint8_t val);
void mrbusCRC16Initialize(void);
#endif

// Global variable externs, so everybody can see the public mrbus variabes
extern volatile uint8_t mrbus_rx_buffer[MRBUS_BUFFER_SIZE];
extern volatile uint8_t mrbus_tx_buffer[MRBUS_BUFFER_SIZE];
extern volatile uint8_t mrbus_state;
extern volatile uint8_t mrbus_priority;
extern volatile uint8_t mrbus_activity;

uint8_t mrbusArbBitSend(uint8_t bitval);
uint8_t mrbusPacketTransmit(void);
void mrbusInit(void);

#endif

// Common macros for handling 16 bit variables
// These aren't strictly part of MRBus, but are used through the code and need to be defined
#ifndef UINT16_HIGH_BYTE
#define UINT16_HIGH_BYTE(a)  ((a)>>8)
#endif 

#ifndef UINT16_LOW_BYTE
#define UINT16_LOW_BYTE(a)  ((a) & 0xFF)
#endif 

#ifndef min
#define min(a,b)  ((a)<(b)?(a):(b))
#endif

#ifndef max
#define max(a,b)  ((a)>(b)?(a):(b))
#endif


#ifdef __cplusplus
}
#endif

