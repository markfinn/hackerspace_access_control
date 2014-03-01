#ifndef MRBUS_CONSTANTS_H
#define MRBUS_CONSTANTS_H

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

// mrbus_activity states
#define MRBUS_ACTIVITY_IDLE          0
#define MRBUS_ACTIVITY_RX            1
#define MRBUS_ACTIVITY_RX_COMPLETE   2

// Specification-defined EEPROM Addresses
#define MRBUS_EE_DEVICE_ADDR         0
#define MRBUS_EE_DEVICE_OPT_FLAGS    1
#define MRBUS_EE_DEVICE_UPDATE_H     2
#define MRBUS_EE_DEVICE_UPDATE_L     3

//MRB-CMP definitions
#define MRBUS_CMP_TYPE 0xff
#define MRBUS_CMP_REPLY_TYPE 0xfe

// Version flags
#define MRBUS_VERSION_WIRELESS 0x80
#define MRBUS_VERSION_WIRED    0x00

#define MRBUS_BAUD   57600

#endif
